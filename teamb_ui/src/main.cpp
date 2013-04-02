#include <QtGui>
#include <QApplication>
#include "../include/main_window.hpp"

int main(int argc, char **argv) {
	
	//Master URL Needed to connect
	//const std::string master_url = "http://localhost:11311";
    //const std::string host_url = "";
    //std::map<std::string,std::string> remappings;

    //ros::init(argc,argv,"teamb_ui");

    //remappings["__master"] = master_url;
    //remappings["__hostname"] = host_url;
    //ros::init(remappings,"team_ui");

    QApplication app(argc, argv);
    teamb_ui::MainWindow w(argc,argv);
    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

	return app.exec();;
}
