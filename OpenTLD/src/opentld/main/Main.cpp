/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/
/*
 * MainX.cpp
 *
 *  Created on: Nov 17, 2011
 *      Author: Georg Nebehay
 */

#include "Main.h"

#include "Config.h"
#include "ImAcq.h"
#include "Gui.h"
#include "TLDUtil.h"

using namespace tld;
Mat* greyRef;
FILE *resultsFile = NULL;

void Main::doWork(OpenTLD::Dest userDest, Mat img)
{

    //IplImage *img = imAcqGetImg(imAcq);
	static Mat grey(img.rows, img.cols, CV_8UC1);
	greyRef = &grey;
    cvtColor(img, grey, CV_BGR2GRAY);

    tld->detectorCascade->imgWidth = grey.cols;
    tld->detectorCascade->imgHeight = grey.rows;
    tld->detectorCascade->imgWidthStep = grey.step;

    if(userDest.destPresent == true)
    {
    	printf("Came here");
        CvRect box;

        if(initialBB == NULL)
        {
            initialBB = new int[4];
        }

        if(box.width == 0 || box.height == 0)
        {
        	printf("Some problem with the bounding box selection");
        	box.width = 50;
        	box.height = 50;
        }

        initialBB[0] = userDest.destX;
        initialBB[1] = userDest.destY;
        initialBB[2] = userDest.destWidth;
        initialBB[3] = userDest.destHeight;
        userDest.destPresent = false;
        printf("W, H: %d %d", initialBB[2], initialBB[3]);
    }



    if(printResults != NULL)
    {
        resultsFile = fopen(printResults, "w");
    }

    bool reuseFrameOnce = false;
    bool skipProcessingOnce = false;


    if(initialBB != NULL)
    {
        Rect bb = tldArrayToRect(initialBB);

        printf("Starting at %d %d %d %d\n", bb.x, bb.y, bb.width, bb.height);

        tld->selectObject(grey, &bb);
        skipProcessingOnce = true;
        reuseFrameOnce = true;
    }
    ROS_INFO("Initialized");
}


OpenTLD::Dest Main::destTrack(Mat* imgRef)
{
	OpenTLD::Dest curDest;
	double tic = cvGetTickCount();

	//cvtColor(*imgRef, *greyRef, CV_BGR2GRAY);

	tld->processImage(*imgRef);

	if(tld->currBB != NULL)
	{
		curDest.destPresent = true;
		curDest.destX = tld->currBB->x;
		curDest.destY = tld->currBB->y;
		curDest.destWidth = tld->currBB->width;
		curDest.destHeight = tld->currBB->height;
		ROS_INFO("%.2d %.2d %.2d %.2d %f\n", tld->currBB->x, tld->currBB->y, tld->currBB->width, tld->currBB->height, tld->currConf);
	}
	else
	{
		curDest.destPresent = false;
		curDest.destX = -1;
		curDest.destY = -1;
		curDest.destWidth = -1;
		curDest.destHeight = -1;
		ROS_INFO("NaN NaN NaN NaN NaN\n");
	}

	double toc = (cvGetTickCount() - tic) / cvGetTickFrequency();

	toc = toc / 1000000;

	float fps = 1 / toc;

	int confident = (tld->currConf >= threshold) ? 1 : 0;

	if(showOutput)
	{
		char string[128];

		char learningString[10] = "";

		if(tld->learning)
		{
			strcpy(learningString, "Learning");
		}

		sprintf(string, "#%d,Posterior %.2f; fps: %.2f, #numwindows:%d, %s", imAcq->currentFrame - 1, tld->currConf, fps, tld->detectorCascade->numWindows, learningString);
		CvScalar yellow = CV_RGB(255, 255, 0);
		CvScalar blue = CV_RGB(0, 0, 255);
		CvScalar black = CV_RGB(0, 0, 0);
		CvScalar white = CV_RGB(255, 255, 255);

		if(tld->currBB != NULL)
		{
			CvScalar rectangleColor = (confident) ? blue : yellow;
			rectangle(*imgRef, tld->currBB->tl(), tld->currBB->br(), rectangleColor, 8, 8, 0);
		}

		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, .5, .5, 0, 1, 8);
		rectangle(*imgRef, cvPoint(0, 0), cvPoint(imgRef->cols, 50), black, CV_FILLED, 8, 0);
		putText(*imgRef, string, Point(25,25), 1, 1.0, white, 2, 1, false);

		if(showForeground)
		{

			for(size_t i = 0; i < tld->detectorCascade->detectionResult->fgList->size(); i++)
			{
				Rect r = tld->detectorCascade->detectionResult->fgList->at(i);
				rectangle(*imgRef, r.tl(), r.br(), white, 1);
			}
		}
	}
	return curDest;
}
