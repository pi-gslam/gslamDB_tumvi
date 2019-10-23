#include <GSLAM/core/GSLAM.h>
#include <string.h>
#include "IO.h"

using namespace GSLAM;
using namespace std;

class FrameIMUTUMVI : public MapFrame
{
public:
    FrameIMUTUMVI(FrameID id,double time,
             Point3d acc ,Point3d angularV,
             Point3d accN,Point3d gyrN,SE3 imu2body=SE3())
           : MapFrame(id,time),
             _acc(acc),_angularV(angularV),
             _accN(accN),_gyrN(gyrN),_i2b(imu2body){}

    virtual std::string type() const{return "FrameIMU";}
    virtual int     getIMUNum()const{return 1;}
    virtual bool    getAcceleration(Point3d& acc,int idx=0)const{acc=_acc;return true;}        // m/s^2
    virtual bool    getAccelerationNoise(Point3d &accN, int idx) const{accN=_accN;return true;}
    virtual bool    getAngularVelocity(Point3d& angularV,int idx=0)const{angularV=_angularV;return true;}// rad/s
    virtual bool    getAngularVNoise(Point3d &angularVN, int idx) const{angularVN=_gyrN;return true;}

    Point3d _acc,_angularV,_accN,_gyrN;
    SE3     _i2b;
};

class FrameMonoTUMVI : public MapFrame
{
public:
    FrameMonoTUMVI(FrameID id,double time,
                   const GImage& img,const Camera& camera,
                   const SE3& camera2body)
        : MapFrame(id,time),_imgLeft(img),_camLeft(camera),_c2bLeft(camera2body){}

    virtual std::string type() const{return "FrameMonoTUMVI";}

    virtual int    cameraNum()const{return 1;}     // Camera number
    virtual GImage getImage(int idx,int channels){return _imgLeft;} // 0:origin image
    virtual Camera getCamera(int idx=0){return _camLeft;}
    virtual SE3    getCameraPose(int idx) const{return _c2bLeft;}
    virtual int    imageChannels(int idx=0) const{return IMAGE_GRAY;}

protected:
    GImage       _imgLeft;
    Camera       _camLeft;
    SE3          _c2bLeft;
};

class FrameStereoTUMVI : public FrameMonoTUMVI{
public:
    FrameStereoTUMVI(FrameID id,double timestamp,
                     const GImage& imgLeft,const GImage& imgRight,
                     const Camera& cameraLeft,const Camera& cameraRight,
                     const SE3&    poseLeft,const SE3& poseRight)
        : FrameMonoTUMVI(id,timestamp,imgLeft,cameraLeft,poseLeft),
          _imgRight(imgRight),_camRight(cameraRight),_c2bRight(poseRight){}


    virtual std::string type() const{return "FrameStereoTUMVI";}

    virtual int    cameraNum()const{return 2;}     // Camera number
    virtual GImage getImage(int idx,int channels){return idx==0?_imgLeft:_imgRight;} // 0:origin image
    virtual Camera getCamera(int idx=0){return idx==0?_camLeft:_camRight;}
    virtual SE3    getCameraPose(int idx) const{return idx==0?_c2bLeft:_c2bRight;}

    GImage       _imgRight;
    Camera       _camRight;
    SE3          _c2bRight;
};

class DatasetTUMVI : public Dataset{
public:
    DatasetTUMVI():curID(1){}
    virtual std::string type() const{return "DatasetTUMVI";}
    virtual bool        isOpened(){return nextImageFrame||nextIMUFrame;}

    static std::string getFolderPath(const std::string& path) {
      auto idx = std::string::npos;
      if ((idx = path.find_last_of('/')) == std::string::npos)
        idx = path.find_last_of('\\');
      if (idx != std::string::npos)
        return path.substr(0, idx);
      else
        return "";
    }

    static std::string getBaseName(const std::string& path) {
      std::string filename = getFileName(path);
      auto idx = filename.find_last_of('.');
      if (idx == std::string::npos)
        return filename;
      else
        return filename.substr(0, idx);
    }

    static std::string getFileName(const std::string& path) {
      auto idx = std::string::npos;
      if ((idx = path.find_last_of('/')) == std::string::npos)
        idx = path.find_last_of('\\');
      if (idx != std::string::npos)
        return path.substr(idx + 1);
      else
        return path;
    }

    virtual bool        open(const std::string& dataset)
    {
        dirtop=getFolderPath(dataset);
        std::string basename=getBaseName(dataset);

//        if(basename!="mono"&&basename!="Monocular")
//            cam1 =loadCamera(cam1Sensor);

        ifs0.open(dirtop+"/cam0/data.csv");
        ifs1.open(dirtop+"/cam1/data.csv");
        std::string line;
//        if(ifs0.is_open()) getline(ifs0,line);
//        else return false;

        if(ifs1.is_open()) getline(ifs1,line);

        ifsIMU.open(dirtop+"/imu0/data.csv");

        if(ifsIMU.is_open()) getline(ifsIMU,line);
        else return false;

        nextImageFrame=grabImageFrame();
        nextIMUFrame  =grabIMUFrame();

        bool ret=nextImageFrame&&nextIMUFrame;
        return ret;
    }

    GSLAM::FramePtr grabIMUFrame(){
        std::string line;
        double      time,rx,ry,rz,ax,ay,az;
        if(!std::getline(ifsIMU,line)) return nullptr;
        sscanf(line.c_str(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf",
               &time,&rx,&ry,&rz,&ax,&ay,&az);

        return GSLAM::FramePtr(new FrameIMUTUMVI(curID++,time*1e-9,Point3d(ax,ay,az),Point3d(rx,ry,rz),accNoise,gyrNoise));
    }

    GSLAM::FramePtr grabImageFrame(){
        std::string line;
        if(!std::getline(ifs1,line)) return GSLAM::FramePtr();

        int idx=line.find_first_of(',');
        if(idx==std::string::npos) return GSLAM::FramePtr();
        double nextImageTime=std::stod(line.substr(0,idx))*1e-9;
        std::string nextImage=line.substr(0,idx)+".png";

        GSLAM::GImage img0=imread(dirtop+"/cam0/data/"+nextImage);
        if(img0.empty())
            LOG(ERROR)<<"Failed to open image "<<nextImage;

        if(img0.cols==512&&!cam0.isValid()){
            cam0=cam512_0;
            cam1=cam512_1;
            cam0P=cam512_i_c0;
            cam1P=cam512_i_c1;
        }
        else if(!cam0.isValid()){

        }

        if(cam1.isValid())
        {
            GSLAM::GImage img1=imread(dirtop+"/cam1/data/"+nextImage);
            return FramePtr(new FrameStereoTUMVI(curID++,nextImageTime,img0,img1,cam0,cam1,cam0P,cam1P));
        }

        return FramePtr(new FrameMonoTUMVI(curID++,nextImageTime,img0,cam0,cam0P));
    }

    virtual GSLAM::FramePtr grabFrame(){
        if(!nextImageFrame||!nextIMUFrame) return FramePtr();
        GSLAM::FramePtr result;
        if(nextImageFrame->timestamp()<nextIMUFrame->timestamp()){
            result=nextImageFrame;
            nextImageFrame=grabImageFrame();
//            LOG(INFO)<<"Frame:"<<result->timestamp();
        }
        else {
            result=nextIMUFrame;
            nextIMUFrame=grabIMUFrame();
        }
        return result;
    }

    Camera cam512_0=Camera({512, 512,190.97847715128717, 190.9733070521226,254.93170605935475, 256.8974428996504,0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182,0});
    Camera cam512_1=Camera({512, 512,190.44236969414825, 190.4344384721956,252.59949716835982, 254.91723064636983,0.0034003170790442797, 0.001766278153469831, -0.00266312569781606, 0.0003299517423931039,0});
    SE3    cam512_i_c0=SE3(0.0453656607259,-0.0719960019392,-0.0447818071239,0.0132723165788,0.694725617491,-0.719111722761,0.00764801952284).inverse();
    SE3    cam512_i_c1=SE3(-0.0556660265249,-0.0701022461789,-0.0475471045069,0.0134102601601,0.71140347541,-0.702611903198,0.007859617077).inverse();

    Point3d gyrNoise=Point3d(0.000080,0.0000022,0); // Point3d(gyroscope_noise_density,gyroscope_random_walk,0)
    Point3d accNoise=Point3d(0.0014,0.000086,0); //(accelerometer_noise_density,accelerometer_random_walk,0);

    std::string    dirtop;
    GSLAM::Camera  cam0,cam1;
    GSLAM::SE3     cam0P,cam1P,imu0P;// Treat IMU as body
    std::ifstream  ifs0,ifs1,ifsIMU;

    GSLAM::FrameID curID;
    FramePtr       nextImageFrame,nextIMUFrame;
};

GSLAM_REGISTER_DATASET(DatasetTUMVI,tumvi);
