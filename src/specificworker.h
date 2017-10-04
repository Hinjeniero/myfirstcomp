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
#define MAXROT 0.9
#define MINDISTANCE 50

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	void setPick(const RoboCompRCISMousePicker::Pick& pick);
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute(); 	

private:
	InnerModel *inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
	struct BState state
	{
	    //--attributes--//
	    bool target;
	    float x;
	    float z;
	    float alpha;
	    //--methods--//
	    bool isEmpty(){return !target;} 
	    void setEmpty(){target=false;}
	    void setTarget(){target=true;}
	    float getX(){return x;}
	    float getZ(){return z;}
	    float getAlpha(){return alpha;}
	    float setX(int _x){x=_x;}
	    float setZ(int _z){z=_z;}
	    float setAlpha(int _alpha){alpha=_alpha;}
	};
};

#endif

