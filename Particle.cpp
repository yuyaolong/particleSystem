//
//  Particle.cpp
//  assignment2
//
//  Created by yuyaolong on 15/9/19.
//  Copyright (c) 2015年 yuyaolong. All rights reserved.
//

#include "Particle.h"


Particle::Particle(const Vector3d &p, const Vector3d &v, const Vector4d &c, const double m, const double l, const double s, const bool t):
    position(p),
    velocity(v),
    color(c),
    mass(m),
    lifeSpan(l),
    pointSize(s),
    stopSign(t)
{
    
}



Particle::Particle()
{
    velocity.set(0,0,0);
    position.set(0,0,0);
    color.set(1,1,1,1);
    mass = 1;
    lifeSpan = 10;
    pointSize = 0.5;
    stopSign = false;
}

void Particle::setPosition(const Vector3d &p)
{
    position = p;
}

void Particle::setVelocity(const Vector3d &v)
{
    velocity = v;
}

void Particle::setColor(const Vector4d &c)
{
    color = c;
}

void Particle::setMass(const double m)
{
    mass = m;
}

void Particle::setLifeSpan(const double l)
{
    lifeSpan = l;
}

void Particle::setPointSize(const double s)
{
    pointSize = s;
}

void Particle::setStopSign(const bool t)
{
    stopSign = t;
}


const Vector3d& Particle::getPosition()
{
    return position;
}

const Vector3d& Particle::getVelocity()
{
    return velocity;
}

const Vector4d& Particle::getColor()
{
    return color;
}

const double Particle::getMass()
{
    return mass;
}

const double Particle::getLifeSpan()
{
    return lifeSpan;
}
const double Particle::getPointSize()
{
    return pointSize;
}

const bool Particle::getStopSign()
{
    return stopSign;
}
