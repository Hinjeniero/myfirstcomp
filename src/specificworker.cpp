/*
 *    Copyright (C) 2017 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	inner = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
    try
    {
	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
	std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	int threshold = 200;
	switch(robotState) {
	  case State::IDLE:
	  {
	    if (!target.isEmpty())
	    robotState = State::GOTO;
	    break;
	  }
	  case State::GOTO:
	  {
	    RoboCompDifferentialRobot::TBaseState state;
	    differentialrobot_proxy->getBaseState(state);
	    inner->updateTransformValues("base", state.x, 0, state.z, 0, state.alpha, 0); //Transforms the robot's vector to the world's point of view. (Or vice versa, don't remember).	    
	    std::pair<float, float> t = target.getTarget();
	    QVec tR = inner->transform("base", QVec::vec3(t.first, 0, t.second), "world"); //Vector's source is robot's location, vector's end is the mouse pick
	    float d = tR.norm2(); //Gets the distance, that equals the vector's module
	    if (d > MINDISTANCE) {
	      float adv;
	      float rot = atan2(tR.x(), tR.z());
	      if (rot > MAXROT) rot = MAXROT;
	      adv = MAXVEL * getSigmoid(d) * getGauss(rot, 0.3, 0.5);
	      std::cout << "-------------------------"<< endl;
	      std::cout << "Sigmoid - " << getSigmoid(d) << endl;
	      std::cout << "Gauss - " << getGauss(rot, 0.3, 0.5) << endl;
	      std::cout << "Vel is - " << adv << endl;
	      std::cout << "Rotation is - " << rot << endl;
	      std::cout << "Distance is - " << d << endl;
	      std::cout << "-------------------------"<< endl;
	      differentialrobot_proxy->setSpeedBase(adv, rot);
	      /**/
	      if(ldata[20].dist < threshold){
		robotState = State::TURN;
	      }
	    } else {
	      differentialrobot_proxy->setSpeedBase(0, 0);
	      target.setEmpty();
	      robotState = State::END;
	    }	    
	    break;
	  }
	  case State::TURN:
	    std::cout << "TURN STATE!" << endl;
	    robotState = State::AVOID;
	    break;
	  case State::AVOID:
	    std::cout << "AVOID STATE!" << endl;
	    robotState = State::GOTO;
	    break;
	  case State::END:
	    std::cout << "END STATE!" << endl;
	    robotState = State::IDLE;
	    break;
	}
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}

float SpecificWorker::getGauss(float Vr, float Vx, float h){
  float lambda = -pow(Vx, 2)/log(h);
  return pow(EulerConstant, (-pow(Vr, 2)/lambda));
}

float SpecificWorker::getSigmoid(float distance){
  //float x = (1/1+pow(EulerConstant, -(distance/500))) -0.5;
  //return 1-(x-1); //hard coded fix, not a good solution
  float x = distance / 100;  //In sigmoid function, the changes on the curve are around 1-2-3 in the X
  return x / (1 + abs(x)); //"fast" sigmoid function
  
}

void SpecificWorker::setPick(const RoboCompRCISMousePicker::Pick& pick)
{
	target.setTarget(pick.x, pick.z);
	std::cout << "Location x -> " << pick.x << " was chosen." << endl;
	std::cout << "Location z -> " << pick.z << " was chosen." << endl; 
}




/*RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	differentialrobot_proxy->setSpeedBase(400, 0);
	if(ldata[20].dist < threshold)
	{
		//std::cout << ldata.front().dist << std::endl;
 		//differentialrobot_proxy->setSpeedBase(10, rot);
		differentialrobot_proxy->setSpeedBase(0, 1);
		usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
	}*/



// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
