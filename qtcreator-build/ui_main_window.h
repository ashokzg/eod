/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created: Wed Apr 3 20:09:59 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDockWidget>
#include <QtGui/QFormLayout>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "eodimg.hpp"
#include "trkimg.hpp"

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QTabWidget *tab_manager;
    QWidget *tab_status;
    eodImg *graphicsView;
    QWidget *tab;
    trkImg *graphicsView_2;
    QWidget *tab_2;
    eodImg *graphicsView_3;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;
    QDockWidget *dock_status;
    QWidget *dockWidgetContents_2;
    QVBoxLayout *verticalLayout;
    QFrame *frame;
    QPushButton *eStopPB;
    QWidget *layoutWidget;
    QFormLayout *formLayout;
    QLabel *label_4;
    QLabel *lbl_modeStatus;
    QLabel *batteryIndLbl;
    QProgressBar *battProgressBar;
    QListView *view_logging;
    QLabel *label_5;
    QCheckBox *cb_enableManualCtrl;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pb_connect;
    QPushButton *pb_setTarget;
    QPushButton *pb_confirmTracking;
    QPushButton *quit_button;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(891, 704);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindowDesign->sizePolicy().hasHeightForWidth());
        MainWindowDesign->setSizePolicy(sizePolicy);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        tab_manager = new QTabWidget(centralwidget);
        tab_manager->setObjectName(QString::fromUtf8("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 639));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_status = new QWidget();
        tab_status->setObjectName(QString::fromUtf8("tab_status"));
        graphicsView = new eodImg(tab_status);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(0, 0, 721, 691));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(graphicsView->sizePolicy().hasHeightForWidth());
        graphicsView->setSizePolicy(sizePolicy1);
        graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        graphicsView->setAlignment(Qt::AlignCenter);
        graphicsView->setRubberBandSelectionMode(Qt::IntersectsItemShape);
        tab_manager->addTab(tab_status, QString());
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        graphicsView_2 = new trkImg(tab);
        graphicsView_2->setObjectName(QString::fromUtf8("graphicsView_2"));
        graphicsView_2->setGeometry(QRect(0, 0, 511, 511));
        sizePolicy1.setHeightForWidth(graphicsView_2->sizePolicy().hasHeightForWidth());
        graphicsView_2->setSizePolicy(sizePolicy1);
        graphicsView_2->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        graphicsView_2->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        graphicsView_2->setAlignment(Qt::AlignCenter);
        graphicsView_2->setRubberBandSelectionMode(Qt::IntersectsItemShape);
        tab_manager->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        graphicsView_3 = new eodImg(tab_2);
        graphicsView_3->setObjectName(QString::fromUtf8("graphicsView_3"));
        graphicsView_3->setGeometry(QRect(0, 0, 721, 691));
        sizePolicy1.setHeightForWidth(graphicsView_3->sizePolicy().hasHeightForWidth());
        graphicsView_3->setSizePolicy(sizePolicy1);
        graphicsView_3->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        graphicsView_3->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        graphicsView_3->setAlignment(Qt::AlignCenter);
        graphicsView_3->setRubberBandSelectionMode(Qt::IntersectsItemShape);
        tab_manager->addTab(tab_2, QString());

        hboxLayout->addWidget(tab_manager);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 891, 25));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);
        dock_status = new QDockWidget(MainWindowDesign);
        dock_status->setObjectName(QString::fromUtf8("dock_status"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(dock_status->sizePolicy().hasHeightForWidth());
        dock_status->setSizePolicy(sizePolicy2);
        dock_status->setMinimumSize(QSize(351, 620));
        dock_status->setAllowedAreas(Qt::RightDockWidgetArea);
        dockWidgetContents_2 = new QWidget();
        dockWidgetContents_2->setObjectName(QString::fromUtf8("dockWidgetContents_2"));
        verticalLayout = new QVBoxLayout(dockWidgetContents_2);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        frame = new QFrame(dockWidgetContents_2);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy3(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy3);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        eStopPB = new QPushButton(frame);
        eStopPB->setObjectName(QString::fromUtf8("eStopPB"));
        eStopPB->setGeometry(QRect(180, 30, 131, 51));
        eStopPB->setCursor(QCursor(Qt::PointingHandCursor));
        eStopPB->setIconSize(QSize(60, 60));
        layoutWidget = new QWidget(frame);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(30, 160, 281, 52));
        formLayout = new QFormLayout(layoutWidget);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setMouseTracking(true);
        label_4->setAutoFillBackground(false);
        label_4->setLineWidth(2);

        formLayout->setWidget(0, QFormLayout::LabelRole, label_4);

        lbl_modeStatus = new QLabel(layoutWidget);
        lbl_modeStatus->setObjectName(QString::fromUtf8("lbl_modeStatus"));
        lbl_modeStatus->setFrameShape(QFrame::StyledPanel);
        lbl_modeStatus->setLineWidth(0);
        lbl_modeStatus->setMargin(0);

        formLayout->setWidget(0, QFormLayout::FieldRole, lbl_modeStatus);

        batteryIndLbl = new QLabel(layoutWidget);
        batteryIndLbl->setObjectName(QString::fromUtf8("batteryIndLbl"));
        batteryIndLbl->setMouseTracking(true);
        batteryIndLbl->setAutoFillBackground(false);
        batteryIndLbl->setLineWidth(2);

        formLayout->setWidget(1, QFormLayout::LabelRole, batteryIndLbl);

        battProgressBar = new QProgressBar(layoutWidget);
        battProgressBar->setObjectName(QString::fromUtf8("battProgressBar"));
        battProgressBar->setValue(24);

        formLayout->setWidget(1, QFormLayout::FieldRole, battProgressBar);

        view_logging = new QListView(frame);
        view_logging->setObjectName(QString::fromUtf8("view_logging"));
        view_logging->setGeometry(QRect(10, 250, 311, 321));
        QSizePolicy sizePolicy4(QSizePolicy::MinimumExpanding, QSizePolicy::Minimum);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(view_logging->sizePolicy().hasHeightForWidth());
        view_logging->setSizePolicy(sizePolicy4);
        view_logging->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        view_logging->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        view_logging->setAlternatingRowColors(true);
        view_logging->setMovement(QListView::Free);
        view_logging->setProperty("isWrapping", QVariant(true));
        view_logging->setResizeMode(QListView::Fixed);
        view_logging->setWordWrap(true);
        label_5 = new QLabel(frame);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 230, 88, 19));
        label_5->setMouseTracking(true);
        label_5->setAutoFillBackground(false);
        label_5->setLineWidth(2);
        cb_enableManualCtrl = new QCheckBox(frame);
        cb_enableManualCtrl->setObjectName(QString::fromUtf8("cb_enableManualCtrl"));
        cb_enableManualCtrl->setGeometry(QRect(30, 130, 171, 22));
        layoutWidget1 = new QWidget(frame);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(33, 28, 131, 62));
        verticalLayout_2 = new QVBoxLayout(layoutWidget1);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        pb_connect = new QPushButton(layoutWidget1);
        pb_connect->setObjectName(QString::fromUtf8("pb_connect"));
        pb_connect->setEnabled(true);
        QSizePolicy sizePolicy5(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(pb_connect->sizePolicy().hasHeightForWidth());
        pb_connect->setSizePolicy(sizePolicy5);

        verticalLayout_2->addWidget(pb_connect);

        pb_setTarget = new QPushButton(layoutWidget1);
        pb_setTarget->setObjectName(QString::fromUtf8("pb_setTarget"));
        pb_setTarget->setCursor(QCursor(Qt::PointingHandCursor));

        verticalLayout_2->addWidget(pb_setTarget);

        pb_confirmTracking = new QPushButton(frame);
        pb_confirmTracking->setObjectName(QString::fromUtf8("pb_confirmTracking"));
        pb_confirmTracking->setGeometry(QRect(33, 96, 132, 27));

        verticalLayout->addWidget(frame);

        quit_button = new QPushButton(dockWidgetContents_2);
        quit_button->setObjectName(QString::fromUtf8("quit_button"));
        sizePolicy5.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy5);

        verticalLayout->addWidget(quit_button);

        dock_status->setWidget(dockWidgetContents_2);
        MainWindowDesign->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dock_status);

        menubar->addAction(menu_File->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addAction(actionAbout_Qt);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));
        QObject::connect(quit_button, SIGNAL(released()), MainWindowDesign, SLOT(close()));
        QObject::connect(eStopPB, SIGNAL(released()), MainWindowDesign, SLOT(resetToIdleManual()));
        QObject::connect(pb_connect, SIGNAL(released()), MainWindowDesign, SLOT(connectToROS()));
        QObject::connect(pb_setTarget, SIGNAL(released()), MainWindowDesign, SLOT(setTargetInPic()));
        QObject::connect(pb_confirmTracking, SIGNAL(released()), MainWindowDesign, SLOT(confirmTracking()));
        QObject::connect(cb_enableManualCtrl, SIGNAL(stateChanged(int)), MainWindowDesign, SLOT(toggleManualMode(int)));

        tab_manager->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "QRosApp", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        graphicsView->setToolTip(QApplication::translate("MainWindowDesign", "Please wait while video loads!", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindowDesign", "Video Feed", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        graphicsView_2->setToolTip(QApplication::translate("MainWindowDesign", "Please wait while video loads!", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        tab_manager->setTabText(tab_manager->indexOf(tab), QApplication::translate("MainWindowDesign", "Tracking Video", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        graphicsView_3->setToolTip(QApplication::translate("MainWindowDesign", "Please wait while video loads!", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        tab_manager->setTabText(tab_manager->indexOf(tab_2), QApplication::translate("MainWindowDesign", "Disparity Map", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&App", 0, QApplication::UnicodeUTF8));
        dock_status->setWindowTitle(QApplication::translate("MainWindowDesign", "Command Panel", 0, QApplication::UnicodeUTF8));
        eStopPB->setText(QApplication::translate("MainWindowDesign", "STOP/ CLEAR \n"
"TARGET", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        label_4->setToolTip(QApplication::translate("MainWindowDesign", "Gives the Feedback from the robot of the mode.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        label_4->setWhatsThis(QApplication::translate("MainWindowDesign", "Mode Status Label", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
        label_4->setText(QApplication::translate("MainWindowDesign", "Mode Status:", 0, QApplication::UnicodeUTF8));
        lbl_modeStatus->setText(QApplication::translate("MainWindowDesign", "Unassigned!", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        batteryIndLbl->setToolTip(QApplication::translate("MainWindowDesign", "Gives the Feedback from the robot of the mode.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        batteryIndLbl->setWhatsThis(QApplication::translate("MainWindowDesign", "Mode Status Label", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
        batteryIndLbl->setText(QApplication::translate("MainWindowDesign", "Battery Status:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        label_5->setToolTip(QApplication::translate("MainWindowDesign", "Gives the Feedback from the robot of the mode.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        label_5->setWhatsThis(QApplication::translate("MainWindowDesign", "Mode Status Label", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
        label_5->setText(QApplication::translate("MainWindowDesign", "Logging:", 0, QApplication::UnicodeUTF8));
        cb_enableManualCtrl->setText(QApplication::translate("MainWindowDesign", "Enable Joystick", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        pb_connect->setToolTip(QApplication::translate("MainWindowDesign", "Set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        pb_connect->setStatusTip(QApplication::translate("MainWindowDesign", "Clear all waypoints and set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        pb_connect->setText(QApplication::translate("MainWindowDesign", "Connect", 0, QApplication::UnicodeUTF8));
        pb_setTarget->setText(QApplication::translate("MainWindowDesign", "Set Target", 0, QApplication::UnicodeUTF8));
        pb_confirmTracking->setText(QApplication::translate("MainWindowDesign", "Confirm Tracking", 0, QApplication::UnicodeUTF8));
        quit_button->setText(QApplication::translate("MainWindowDesign", "Quit", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
