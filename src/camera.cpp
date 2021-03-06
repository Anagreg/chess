/***************************************************************************
 *   Copyright (C) 2009-2018 by Veselin Georgiev, Slavomir Kaslev,         *
 *                              Deyan Hadzhiev et al                       *
 *   admin@raytracing-bg.net                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
/**
 * @File camera.cpp
 * @Brief Implementation of the raytracing camera.
 */

#include "camera.h"
#include "matrix.h"
#include "util.h"
#include "sdl.h"
#include "random_generator.h"
#include <algorithm>
using namespace std;

void Camera::beginFrame()
{
	Vector C = Vector(-aspectRatio, 1, 1);
	Vector B = Vector(0, 0, 1);
	Vector BC = C - B;
	double lenBC = BC.length();
	double lenWanted = tan(toRadians(fov/2));
	double m = lenWanted / lenBC;
	topLeft    = Vector(-aspectRatio * m, +m, 1);
	topRight   = Vector(+aspectRatio * m, +m, 1);
	bottomLeft = Vector(-aspectRatio * m, -m, 1);
	w = frameWidth();
	h = frameHeight();
	
	Matrix rotation = rotationAroundZ(toRadians(roll)) * rotationAroundX(toRadians(pitch)) *
	                  rotationAroundY(toRadians(yaw));
	topLeft *= rotation;
	topRight *= rotation;
	bottomLeft *= rotation;
	frontDir = Vector(0, 0, 1) * rotation;
	upDir = Vector(0, 1, 0) * rotation;
	rightDir = Vector(1, 0, 0) * rotation;
	apertureSize = 1.0 / fNumber;
}

Ray Camera::getScreenRay(double x, double y, WhichCamera whichCamera)
{
	Ray result;
	result.dir = topLeft + (topRight - topLeft) * (x / w)
	                     + (bottomLeft - topLeft) * (y / h);
	result.dir.normalize();
	result.start = this->pos;
	if (whichCamera != CAMERA_CENTER) {
		if (whichCamera == CAMERA_LEFT)
			result.start += rightDir * -stereoSeparation;
		else
			result.start += rightDir * stereoSeparation;
	}
	return result;
}

Ray Camera::getDOFRay(double x, double y, WhichCamera whichCamera)
{
	Ray ray = getScreenRay(x, y, whichCamera);
	Vector screenRayDir = ray.dir;
	double M = focalPlaneDist / dot(frontDir, screenRayDir);
	Vector T = this->pos + screenRayDir * M;
	
	double u, v;
	Random& rnd = getRandomGen();
	rnd.unitDiscSample(u, v);
	u *= apertureSize;
	v *= apertureSize;
	ray.start += u * rightDir + v * upDir;
	ray.dir = T - ray.start;
	ray.dir.normalize();
	
	return ray;
}


void Camera::move(double rx, double ry)
{
	pos += rx * rightDir + ry * frontDir;
}

void Camera::rotate(double rx, double ry)
{
	yaw += rx;
	pitch += ry;
	pitch = min(pitch, +90.0);
	pitch = max(pitch, -90.0);
}
