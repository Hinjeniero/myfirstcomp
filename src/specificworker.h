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

/**
       \brief
       @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#define MAXVEL 400
#define MAXROT 1
#define MINDISTANCE 50




class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	const float EulerConstant = std::exp(1.0);
	void setPick(const RoboCompRCISMousePicker::Pick& pick);
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	float getGauss(float Vr, float Vx, float h);
	float getSigmoid(float distance);
	enum State {IDLE, GOTO, TURN, AVOID, END};
	State state = State::IDLE;


public slots:
	void compute(); 	

private:
	InnerModel *inner;
	struct Target
	{
	    bool empty = true;
	    float x;
	    float z;
	    QMutex mutex;
	    bool isEmpty()
	    {
	      QMutexLocker ml(&mutex);
	      return empty;
	    } 
	    
	    void setEmpty()
	    {
	      QMutexLocker ml(&mutex);
	      empty = true;
	    };
	    void setTarget(float x_, float z_)
	    {
	      QMutexLocker ml(&mutex);
	      empty = false;
	      x = x_;
	      z = z_;
	    };	    
	    std::pair<float, float> getTarget()
	    {
	      QMutexLocker ml(&mutex);
	      return std::make_pair(x,z);
	    };
	};
	Target target;
};

#endif

