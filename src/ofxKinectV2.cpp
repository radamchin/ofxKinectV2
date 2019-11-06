//
//  ofxKinectV2.cpp
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#include "ofxKinectV2.h"

//static
//void ofxKinectV2::setMinMaxDistance(float minMeters, float maxMeters)
//{
//    ofProtonect::setMinMaxDistance(minMeters, maxMeters);
//}
//
//void ofxKinectV2::setFreenectConfiguration(libfreenect2::Freenect2Device::Config config)
//{
//    ofProtonect::setConfiguration(config);
//}

//non-static
ofxKinectV2::ofxKinectV2()
{
    //set default distance range to 50cm - 600cm
    params.add(minDistance.set("minDistance", 500, 0, 12000));
    params.add(maxDistance.set("maxDistance", 6000, 0, 24000));
    params.add(irExposure.set("irExposure", 1.0, 0.01, 10.0));
    
    
    // RGB cam controls
    params.add(autoExposure.set("Auto Exposure", true));
    autoExposure.addListener(this, &ofxKinectV2::setAutoExposureCallback);
    
    // Below 2 are for adjusting exposure when auto is off
    params.add(expIntegrationTime.set("Shutter speed", 50.0, 0.0, 66.0));
    expIntegrationTime.addListener(this, &ofxKinectV2::setIntegrationTimeCallback);
    
    params.add(analogueGain.set("analogueGain", 3.0, 1.0, 4.0));
    analogueGain.addListener(this, &ofxKinectV2::setAnalogueGainCallback);

    
    params.add(autoWhiteBalance.set("Auto White Balance", true));
    autoWhiteBalance.addListener(this, &ofxKinectV2::setAutoWhiteBalanceCallback);
    
    // Below 3 are for adjusting white balance when auto is off
    params.add(redGain.set("redGain", 2.0, 0.01, 4.0));
    redGain.addListener(this, &ofxKinectV2::setRedGainCallback);
    
    params.add(blueGain.set("blueGain", 2.0, 0.01, 4.0));
    blueGain.addListener(this, &ofxKinectV2::setBlueGainCallback);
    
    params.add(greenGain.set("greenGain", 2.0, 0.01, 4.0));
    greenGain.addListener(this, &ofxKinectV2::setGreenGainCallback);
    
}


ofxKinectV2::~ofxKinectV2()
{
    close();
}


std::vector<ofxKinectV2::KinectDeviceInfo> ofxKinectV2::getDeviceList() const
{
    std::vector<KinectDeviceInfo> devices;
    
    int num = protonect.getFreenect2Instance().enumerateDevices();

    for (int i = 0; i < num; i++)
    {
        KinectDeviceInfo kdi;
        kdi.serial = protonect.getFreenect2Instance().getDeviceSerialNumber(i);
        kdi.freenectId = i; 
        devices.push_back(kdi);
    }
    
    ofSort(devices, [](const ofxKinectV2::KinectDeviceInfo& A,
                       const ofxKinectV2::KinectDeviceInfo& B)
           {
               return A.serial < B.serial;
           });

    for (std::size_t i = 0; i < num; i++)
    {
        devices[i].deviceId = i;
    }
    
    return devices;
}


std::size_t ofxKinectV2::getNumDevices() const
{
   return getDeviceList().size(); 
}


bool ofxKinectV2::open(int deviceId, ofxKinectV2::Settings asettings)
{
    std::vector<KinectDeviceInfo> devices = getDeviceList();
    
    if (devices.empty())
    {
        ofLogError("ofxKinectV2::open") << "no devices connected!";
        return false;
    }
    
    if(deviceId >= devices.size())
    {
        ofLogError("ofxKinectV2::open") << " deviceId " << deviceId << " is bigger or equal to the number of connected devices " << devices.size() << endl;
        return false;
    }

    string serial = devices[deviceId].serial;
    
    return open(serial, asettings);
}


bool ofxKinectV2::open(const std::string& serial, ofxKinectV2::Settings asettings)
{
    close(); 

    params.setName("KinectV2 " + serial);
    
    bNewFrame  = false;
    bNewBuffer = false;
    bOpened    = false;

	mSettings = asettings;
    
	protonect.setRGBEnabled(asettings.enableRGB);
	protonect.setDepthEnabled(asettings.enableDepth);
	protonect.setIREnabled(asettings.enableIR);
	protonect.setEnableRGBRegistration(asettings.enableRGBRegistration);
    int retVal = protonect.open(serial,asettings.pipeline,asettings.config);
    
    if (retVal != 0)
    {
        return false;
    }
    
    lastFrameNo = -1;
    startThread();
    bOpened = true;
    return true;
}


void ofxKinectV2::threadedFunction()
{
    while (isThreadRunning())
    {
        protonect.updateKinect(pixelsBack,
                               registeredPixelsBack,
                               rawDepthPixelsBack,
                               rawIRPixelsBack,
                               distancePixelsBack);
        
        pixelsFront.swap(pixelsBack);
        registeredPixelsFront.swap(registeredPixelsBack);
        rawDepthPixelsFront.swap(rawDepthPixelsBack);
        rawIRPixelsFront.swap(rawIRPixelsBack);
        distancePixelsFront.swap(distancePixelsBack);
        
        lock();
        bNewBuffer = true;
        unlock();
    }
}


void ofxKinectV2::update()
{
    if (ofGetFrameNum() != lastFrameNo)
    {
        bNewFrame = false;
        lastFrameNo = ofGetFrameNum();
    }
    
    if (bNewBuffer)
    {
        lock();
            pixels = pixelsFront;
            registeredPixels = registeredPixelsFront;
            rawDepthPixels = rawDepthPixelsFront;
            rawIRPixels = rawIRPixelsFront;
            bNewBuffer = false;
        unlock();

        // TODO: This is inefficient and we should be able to turn it off or
        // draw it directly with a shader.
        if (rawDepthPixels.size() > 0)
        {
            if (depthPixels.getWidth() != rawDepthPixels.getWidth())
            {
                depthPixels.allocate(rawDepthPixels.getWidth(), rawDepthPixels.getHeight(), 1);
            }
        
            float* pixelsF = rawDepthPixels.getData();
            unsigned char * pixelsC = depthPixels.getData();
                
            for (std::size_t i = 0; i < depthPixels.size(); i++)
            {
                pixelsC[i] = ofMap(pixelsF[i], minDistance, maxDistance, 255, 0, true);
            
                if (pixelsC[i] == 255)
                {
                    pixelsC[i] = 0;
                }
            }
        }
        
        // TODO: This is inefficient and we should be able to turn it off or
        // draw it directly with a shader.
        if (rawIRPixels.size() > 0)
        {
            if (irPixels.getWidth() != rawIRPixels.getWidth())
            {
                irPixels.allocate(rawIRPixels.getWidth(), rawIRPixels.getHeight(), 1);
            }
            
            float* pixelsF = rawIRPixels.getData();
            unsigned char * pixelsC = irPixels.getData();
                        
            for (std::size_t i = 0; i < irPixels.size(); i++)
            {
                //this is an arbitary mapping at the moment.
                //basically exposure. 6000.0 = max brightness seems to be good. 
                pixelsC[i] = std::min(255.0, 255.0 * irExposure * (pixelsF[i]/6000.0));
            }
        }

        bNewFrame = true;
    }
}


bool ofxKinectV2::isFrameNew() const
{
    return bNewFrame; 
}


ofPixels ofxKinectV2::getRgbPixels()
{
    return getPixels();
}


const ofPixels& ofxKinectV2::getPixels() const
{
    return pixels;
}


const ofPixels& ofxKinectV2::getRegisteredPixels() const
{
    return registeredPixels;
}


const ofFloatPixels& ofxKinectV2::getRawDepthPixels() const
{
    return rawDepthPixels;
}


const ofPixels& ofxKinectV2::getDepthPixels() const
{
    return depthPixels;
}


const ofFloatPixels& ofxKinectV2::getRawIRPixels() const
{
    return rawIRPixels;
}


const ofPixels& ofxKinectV2::getIRPixels() const
{
    return irPixels;
}


float ofxKinectV2::getDistanceAt(std::size_t x, std::size_t y) const
{
    return glm::distance(glm::vec3(0, 0, 0), getWorldCoordinateAt(x, y));
}

glm::vec3 ofxKinectV2::getWorldCoordinateAt(std::size_t x, std::size_t y) const
{
    glm::vec3 position;
    
    if (protonect.registration && protonect.undistorted)
    {
//        std::cout << x << ", " << protonect.undistorted->width << std::endl;
//        std::cout << y << ", " << protonect.undistorted->height << std::endl;
//        
        if (x < protonect.undistorted->width && y < protonect.undistorted->height)
            protonect.registration->getPointXYZ(protonect.undistorted, y, x, position.x, position.y, position.z);
        else ofLogWarning("ofxKinectV2::getWorldCoordinateAt") << "Invalid x, y coordinates.";

    }
    else ofLogWarning("ofxKinectV2::getWorldCoordinateAt") << "Kinect is not initialized, returning 0, 0, 0.";
    
    return position;
}

// ------------------------------------------
int ofxKinectV2::getMinDistance() const {
	return (mSettings.config.MinDepth * 1000);
}

// ------------------------------------------
int ofxKinectV2::getMaxDistance() const {
	return (mSettings.config.MaxDepth * 1000);
}

// ------------------------------------------
bool ofxKinectV2::isDepthEnabled() const {
	return mSettings.enableDepth;
}

// ------------------------------------------
bool ofxKinectV2::isIREnabled() const {
	return mSettings.enableIR;
}

// ------------------------------------------
bool ofxKinectV2::isRGBEnabled() const {
	return mSettings.enableRGB;
}

// ------------------------------------------
bool ofxKinectV2::isRGBRegistrationEnabled() const {
	return mSettings.enableRGBRegistration;
}

// ------------------------------------------

void ofxKinectV2::setAutoExposureCallback(bool & auto_exposure){
    if(auto_exposure){
        autoExposure = true;
        protonect.dev->setColorAutoExposure(0);
    }
}

void ofxKinectV2::setIntegrationTimeCallback(float & integration_time_ms){
    protonect.dev->setColorManualExposure(integration_time_ms, analogueGain);
    autoExposure = false;
}

void ofxKinectV2::setAnalogueGainCallback(float & analog_gain){
    protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
    protonect.dev->setColorManualExposure(expIntegrationTime, analog_gain);
    autoExposure = false;
}

void ofxKinectV2::setAutoWhiteBalanceCallback(bool & auto_white_balance){
    if(auto_white_balance == true){
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(1));
    }else{
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(3));
    }
}

void ofxKinectV2::setRedGainCallback(float & red_gain){
    if(autoWhiteBalance){
        autoWhiteBalance = false;
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(3));
    }
    protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_RED_CHANNEL_GAIN, red_gain);
}

void ofxKinectV2::setGreenGainCallback(float & green_gain){
    if(autoWhiteBalance){
        autoWhiteBalance = false;
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(3));
    }
    protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_GREEN_CHANNEL_GAIN, green_gain);
}

void ofxKinectV2::setBlueGainCallback(float & blue_gain){
    if(autoWhiteBalance){
        autoWhiteBalance = false;
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(3));
    }
    protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_BLUE_CHANNEL_GAIN, blue_gain);
}

// ------------------------------------------

void ofxKinectV2::close()
{
    if (bOpened)
    {
        waitForThread(true);
        protonect.closeKinect();
        bOpened = false;
    }
}
