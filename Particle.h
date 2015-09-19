//
//  Particle.h
//  assignment2
//
//  Created by yuyaolong on 15/9/19.
//  Copyright (c) 2015年 yuyaolong. All rights reserved.
//

#ifndef __assignment2__Particle__
#define __assignment2__Particle__

#include <stdio.h>
#include "Vector.h"

class Particle
{
public:
    Particle();
    Particle(const Vector3d &p, const Vector3d &v, const Vector4d &c, const double m, const double l, const double s, const bool t);
    //~Particle();
    //Particle(const Particle&);
    //Particle& operator= (const Particle& );
    
    void setPosition(const Vector3d&);
    void setVelocity(const Vector3d&);
    void setColor(const Vector4d&);
    void setMass(const double);
    void setLifeSpan(const double);
    void setPointSize(const double);
    void setStopSign(const bool);
    
    const Vector3d& getPosition();
    const Vector3d& getVelocity();
    const Vector4d& getColor();
    const double   getMass();
    const double   getLifeSpan();
    const double   getPointSize();
    const bool     getStopSign();
    
private:
    Vector3d position;
    Vector3d velocity;
    Vector4d color;
    double mass;
    int lifeSpan;
    double pointSize;
    bool   stopSign;
    
};

#endif /* defined(__assignment2__Particle__) */
