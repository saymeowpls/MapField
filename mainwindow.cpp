#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QDebug>

#include <QByteArray>
//#include <QString>
#include <QtMath>

#define pi 3.141592653589

char cTmpBuff[512];
char cTmpBuffTemper[16];

QString qsGPSB = "#GPSB";
QString qsGPSE = "#GPSE";


typedef struct sData{
    char chDx[3];
    char chDy[3];
    char chDz[3];
    char chZap1;
    char chNumPack[4];
    char chZap2;
    char chMark;
}sData;

typedef struct sDataTemper{
    char chDT0[2];
    char chDT1[2];
    char chDT2[2];
    char chDT3[2];
    char chZap[2];
    char chNumPack[4];
    char chZap2;
    char chMark;
}sDataTemper;

typedef struct sDataGPS{
//$GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
    double Time_Stamp; //UTC of position fix
    QString qsValidity; //Data status: A=Data Valid V=Data Invalid  (V=navigation receiver warning)
    double dbLatitude; //Latitude of fix
    QString qsNorth_South; //N or S
    double dbLongitude; //Longitude of fix
    QString qsEast_West; //E or W
    double  dbSpeed_in_Knots; //Speed over ground in knots
    double dbCourse; //Track made good in degrees True
    int Date_Stamp; //UT date
    double dbVariation; //Magnetic variation degrees (Easterly var. subtracts from true course)
    QString qsMag_East_West; //E or W
    int Checksum_GPRMC; //Checksum GPRMC
//$GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
    int Fix_Quality; //GPS_Quality_indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
    int Number_of_Satellites; //Number of satellites in use
    double dbHDOP; //Relative accuracy of horizontal position
    double dbAltitude; // Antenna altitude above mean-sea-level
    QString qsUnits; //Units of antenna altitude, meters
    double dbGeoidal; //Height of geoid above WGS84 ellipsoid
    double dbTime_Last_DGPS_Update; //Age of Differential GPS data (seconds)
    int ID_DifStation; //Differential reference station ID
    int Checksum_GPGGA; //Checksum GPGGA
//$PORZD,A,xxx.x,*hh
    QString qsValidityPORZD; //Status: A=Data Valid V=Data Invalid
    double dbRMSE; //RMS error of plane coordinates
    int Checksum_PORZD; //Checksum PORZD
//$GPGBS,hhmmss.s-s,x.x,x.x,x.x,xx,x.x,x.x,x.x,*hh
    double dbLatitude_Error_Est; //Latitude Error Estimation
    double dbLongitude_Error_Est; //Longitude Error Estimation
    double dbAltitude_Error_Est; //Altitude Error Estimation
    int PRN; //PRN of most likely failed and therefore excluded satellite
    double dbExcluded_Measurement_Error; //Excluded Measurement Error Estimate (m)
    double dbStandard_Deviation_Pos; //Standard deviation of position
    int Checksum_GPGBS; //Checksum GPGBS
}sDataGPS;

typedef struct sDataCoordXY{
    int X;
    int Y;
}sDataCoordXY;


void fnRecalcLatLon2XYMerc(struct sDataGPS * sGPS, struct sDataCoordXY * sXY){
    double Lat = sGPS->dbLatitude / 100;
    double Longt = sGPS->dbLongitude / 100;
    double rLat = Lat*pi/180;
    double rLongt = Longt*pi/180;
    double a = 6378137;
    double b = 6356752.3142;
    double f = (a-b)/a;
    double e = sqrt(2*f-f*f);
    sXY->X = a * rLongt;
    sXY->Y = a * log(tan(pi/4+rLat/2)*pow(((1-e*sin(rLat))/(1+e*sin(rLat))),(e/2)));
}

void MainWindow::setupSimpleDemo(QCustomPlot *customPlot)
{
  demoName = "Simple Demo";

  // add two new graphs and set their look:

  customPlot->addGraph();
  /*//////////////////////////////////////////*/


  QCPGraph *graph = customPlot->graph();
  QCPDataSelection selection = graph->selection();
  double sum = 0;
  foreach (QCPDataRange dataRange, selection.dataRanges())
  {
    QCPGraphDataContainer::const_iterator begin = graph->data()->at(dataRange.begin()); // get range begin iterator from index
    QCPGraphDataContainer::const_iterator end = graph->data()->at(dataRange.end()); // get range end iterator from index
    for (QCPGraphDataContainer::const_iterator it=begin; it!=end; ++it)
    {
      // iterator "it" will go through all selected data points, as an example, we calculate the value average
      sum += it->value;
    }
  }
  double average = sum/selection.dataPointCount();

//  customPlot->setInteraction(QCP::iSelectPlottables, true);
//  customPlot->setInteraction(QCP::iMultiSelect, true);

  customPlot->plottable()->setSelectable(QCP::stSingleData); // vrode kak to

//  customPlot->setSelectionRectMode(QCP::srmNone);
//  customPlot->setSelectionType(QCP::stMultipleDataRanges)
//  customPlot->setSelectionType(QCP::stSingleData, true);


 /* /////////////////////////////////////////////////////*/

  customPlot->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
  customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
  customPlot->addGraph();
  customPlot->graph(1)->setPen(QPen(Qt::red)); // line color red for second graph
  // generate some points of data (y0 for first, y1 for second graph):
  QVector<double> x(251), y0(251), y1(251);
  for (int i=0; i<251; ++i)
  {
    x[i] = i;
    y0[i] = qExp(-i/150.0)*qCos(i/10.0); // exponentially decaying cosine
    y1[i] = qExp(-i/150.0);              // exponential envelope
  }
  // configure right and top axis to show ticks but no labels:
  // (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)
  customPlot->xAxis2->setVisible(true);
  customPlot->xAxis2->setTickLabels(false);
  customPlot->yAxis2->setVisible(true);
  customPlot->yAxis2->setTickLabels(false);
  // make left and bottom axes always transfer their ranges to right and top axes:
  connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
  connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));
  // pass data points to graphs:
  customPlot->graph(0)->setData(x, y0);
  customPlot->graph(1)->setData(x, y1);
  // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
  customPlot->graph(0)->rescaleAxes();
  // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
  customPlot->graph(1)->rescaleAxes(true);
  // Note: we could have also just called customPlot->rescaleAxes(); instead
  // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
  customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables | QCP::iMultiSelect);
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setupSimpleDemo(ui->Ox);
    setupSimpleDemo(ui->Oy);
    setupSimpleDemo(ui->Oz);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadFile(const QString &fileName)
{
    qDebug() << "filename!!!" << fileName;


    QFile file(fileName);
    QFile file1("output.csv");

    if ( !file.open(QFile::ReadWrite) ) {
        qDebug() << "File not exists2";
    }
    else
        qDebug() << "working!";


    if ( !file1.open(QFile::ReadWrite | QFile::Text) ) {
        qDebug() << "File not exists1";
    }


    QTextStream out(&file1);
    int numbl = 0;
    file.seek(2048);
    sData sTmpData;
    bool flagGPS = false;
    QVector<sDataGPS> vGPS;
    QVector<sDataCoordXY> vCoordXY;
    while (!file.atEnd()) {
        /*int res = */file.read(&cTmpBuff[0], 512);
        QByteArray qbTmpBuff(cTmpBuff,512);
        QString qsTmpBuffStr(qbTmpBuff);
        int num;
        if((num = qsTmpBuffStr.indexOf(qsGPSB, 0)) == -1){
            numbl++;
//                qDebug() << numbl << "\n";
            flagGPS = false;
            for(int i=0; i<496; i+=16){
//                    qDebug() << i << "\n";
                memcpy((void*)(&(sTmpData.chDx[0])), &cTmpBuff[i], 16);
                int tmp1=0, tmp2=0, tmp3=0, tmp4=0;
                memcpy((void*)(&tmp1),&(sTmpData.chDx[0]),3);
                memcpy((void*)(&tmp2),&(sTmpData.chDy[0]),3);
                memcpy((void*)(&tmp3),&(sTmpData.chDz[0]),3);
                memcpy((void*)(&tmp4),&(sTmpData.chNumPack[0]),4);
                tmp1 = tmp1 << 8;
                tmp1 = tmp1 >> 8;
                tmp2 = tmp2 << 8;
                tmp2 = tmp2 >> 8;
                tmp3 = tmp3 << 8;
                tmp3 = tmp3 >> 8;
//                    qDebug() << uint8_t(sTmpData.chDx[2]) << "," <<  uint8_t(sTmpData.chDy[2]) << "," << uint8_t(sTmpData.chDz[2]) << "," << uint8_t(sTmpData.chNumPack[0]) << "," << uint8_t(sTmpData.chMark) << "\n";
//                    qDebug() << tmp1 << "," << tmp2 << "," << tmp3 << "," << tmp4 << "," << (uint8_t)(sTmpData.chMark) << "\n";
                out << tmp1 << "," << tmp2 << "," << tmp3 << "," << tmp4 << "," << (uint8_t)(sTmpData.chMark) << "\n";
             }
             memcpy((void*)(&cTmpBuffTemper[0]), &cTmpBuff[496], 16);
             sDataTemper sTmpDataTemper;
             memcpy((void*)(&(sTmpDataTemper.chDT0[0])), &cTmpBuffTemper[0], 16);
             int tmp1=0, tmp2=0, tmp3=0, tmp4=0, tmp5=0;
             memcpy((void*)(&tmp1),&(sTmpDataTemper.chDT0[0]),2);
             memcpy((void*)(&tmp2),&(sTmpDataTemper.chDT1[0]),2);
             memcpy((void*)(&tmp3),&(sTmpDataTemper.chDT2[0]),2);
             memcpy((void*)(&tmp4),&(sTmpDataTemper.chDT3[0]),2);
             memcpy((void*)(&tmp5),&(sTmpDataTemper.chNumPack[0]),4);
//                 qDebug() << tmp1 << "," << tmp2 << "," << tmp3 << "," << tmp4 << "," << tmp5 << "," << (uint8_t)(sTmpDataTemper.chMark) << "\n";
        }
        else{
            flagGPS = true;
            sDataGPS sGPS;
            QByteArray qbTmpBuff1;
            for(int i=16; i<512; i++){
                qbTmpBuff1.push_back(qbTmpBuff.at(i));
            }
            QString qsTmpBuffStr1(qbTmpBuff1);

            QStringList list = qsTmpBuffStr1.split(",");
            for (int i=0; i<list.size(); ++i){
                QString tmpqstr=list.at(i);
                if(tmpqstr.contains("$GNRMC",Qt::CaseInsensitive)){
//                        qDebug() << "$GNRMC" << "\n";
                    QString tmp;
                    tmp = list.at(i+1);
                    sGPS.Time_Stamp = tmp.toDouble();
                    sGPS.qsValidity = list.at(i+2);
                    tmp = list.at(i+3);
                    sGPS.dbLatitude = tmp.toDouble();
                    sGPS.qsNorth_South = list.at(i+4);
                    tmp = list.at(i+5);
                    sGPS.dbLongitude = tmp.toDouble();
                    sGPS.qsEast_West = list.at(i+6);
                    tmp = list.at(i+7);
                    sGPS.dbSpeed_in_Knots = tmp.toDouble();
                    tmp = list.at(i+8);
                    sGPS.dbCourse = tmp.toDouble();
                    tmp = list.at(i+9);
                    sGPS.Date_Stamp = tmp.toInt();
                    tmp = list.at(i+10);
                    sGPS.dbVariation = tmp.toDouble();
                    sGPS.qsMag_East_West = tmp = list.at(i+11);
                    break;
                }
            }
            for (int i=0; i<list.size(); ++i){
                QString tmpqstr=list.at(i);
                if(tmpqstr.contains("$GNGGA",Qt::CaseInsensitive)){
//                        qDebug() << "$GNGGA" << "\n";
                    QString tmp;
                    tmp = list.at(i+6);
                    sGPS.Fix_Quality = tmp.toInt();
                    tmp = list.at(i+7);
                    sGPS.Number_of_Satellites = tmp.toInt();
                    tmp = list.at(i+8);
                    sGPS.dbHDOP = tmp.toDouble();
                    tmp = list.at(i+9);
                    sGPS.dbAltitude = tmp.toDouble();
                    sGPS.qsUnits = list.at(i+10);
                    tmp = list.at(i+11);
                    sGPS.dbGeoidal = tmp.toDouble();
                    tmp = list.at(i+12);
                    sGPS.dbTime_Last_DGPS_Update = tmp.toDouble();
                    tmp = list.at(i+13);
                    sGPS.ID_DifStation = tmp.toInt();
                    break;
                }
            }
            for (int i=0; i<list.size(); ++i){
                QString tmpqstr=list.at(i);
                if(tmpqstr.contains("$PORZD",Qt::CaseInsensitive)){
//                        qDebug() << "$PORZD" << "\n";
                    QString tmp;
                    sGPS.qsValidityPORZD = list.at(i+1);
                    tmp = list.at(i+2);
                    sGPS.dbRMSE = tmp.toDouble();
                    break;
                }
            }
            for (int i=0; i<list.size(); ++i){
                QString tmpqstr=list.at(i);
                if(tmpqstr.contains("$GNGBS",Qt::CaseInsensitive)){
//                        qDebug() << "$GNGBS" << "\n";
                    QString tmp;
                    tmp = list.at(i+2);
                    sGPS.dbLatitude_Error_Est = tmp.toDouble();
                    tmp = list.at(i+3);
                    sGPS.dbLongitude_Error_Est = tmp.toDouble();
                    tmp = list.at(i+4);
                    sGPS.dbAltitude_Error_Est = tmp.toDouble();
                    tmp = list.at(i+5);
                    sGPS.PRN = tmp.toInt();
                    tmp = list.at(i+6);
                    sGPS.dbExcluded_Measurement_Error = tmp.toDouble();
                    tmp = list.at(i+7);
                    sGPS.dbStandard_Deviation_Pos = tmp.toDouble();
                    break;
                }
            }
            vGPS.push_back(sGPS);
            sDataCoordXY sXY;
            sDataCoordXY tmpXY;
            tmpXY.X=0;
            tmpXY.Y=0;
//            int a = 1;
            fnRecalcLatLon2XYMerc(&sGPS, &sXY);
            vCoordXY.push_back(sXY);
            if (sXY.X != tmpXY.X || sXY.Y != tmpXY.Y)
            {
                qDebug("Time_Stamp: %10.3f\n", sGPS.Time_Stamp);
                qDebug("X: %d Y: %d\n",  sXY.X, sXY.Y);
                tmpXY.X = sXY.X;
                tmpXY.Y = sXY.Y;
            }
//                qDebug() << "#GPSB" << "\n";
        }
    }
    qDebug() << "fin" << "\n";
    file.close();
    file1.close();
    ui->statusbar->showMessage("File loaded");

}

void MainWindow::on_actionOpen_triggered()
{
    ui->statusbar->showMessage("Loading file");
    QString fileName = QFileDialog::getOpenFileName(0, "Open Dialog", "", "*.f16");

    if (!fileName.isEmpty())
        loadFile(fileName);
}
