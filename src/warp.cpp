/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Vector3f Warp::sampleUniformHemisphere(Sampler *sampler, const Normal3f &pole) {
    // Naive implementation using rejection sampling
    Vector3f v;
    do {
        v.x() = 1.f - 2.f * sampler->next1D();
        v.y() = 1.f - 2.f * sampler->next1D();
        v.z() = 1.f - 2.f * sampler->next1D();
    } while (v.squaredNorm() > 1.f);

    if (v.dot(pole) < 0.f)
        v = -v;
    v /= v.norm();

    return v;
}


Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}


Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    auto r = sqrt(sample.x());
    auto theta = 2.0f * M_PI * sample.y();
    return Point2f(r * cos(theta), r * sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return p.norm() < 1.0f ? INV_PI : 0.0f;
}


Vector3f Warp::squareToUniformCylinder(const Point2f &sample) {
    auto phi = 2.0f * M_PI * sample.y();
    return Vector3f{cos(phi), sin(phi), 2.0f * sample.x() - 1.0f};
}


Vector3f Warp::squareToUniformSphereCap(const Point2f &sample, float cosThetaMax) {
    auto capHeight = 1.0f - cosThetaMax;
    auto cylinderPoint = squareToUniformCylinder(sample);
    auto z = abs(cylinderPoint.z() * capHeight) + cosThetaMax;
    auto r = sqrt(1.0f - powf(z, 2));
    return Vector3f{r * cylinderPoint.x(), r * cylinderPoint.y(), z};
}

float Warp::squareToUniformSphereCapPdf(const Vector3f &v, float cosThetaMax) {
    return abs(1.0f - v.norm()) < Epsilon && v.z() >= cosThetaMax ?
        INV_TWOPI / (1.0f - cosThetaMax) : 0.0f;
}


Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    auto cylinderPoint = squareToUniformCylinder(sample);
    auto r = sqrt(1.0f - powf(cylinderPoint.z(), 2));
    return Vector3f{r * cylinderPoint.x(), r * cylinderPoint.y(), cylinderPoint.z()};
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return abs(1.0f - v.norm()) < Epsilon ? INV_FOURPI : 0.0f;
}


Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    auto cylinderPoint = squareToUniformCylinder(sample);
    auto r = sqrt(1.0f - powf(cylinderPoint.z(), 2));
    return Vector3f{r * cylinderPoint.x(), r * cylinderPoint.y(), abs(cylinderPoint.z())};
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return v.z() >= 0.0f && abs(1.0f - v.norm()) < Epsilon ? INV_TWOPI : 0.0f;
}


Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    auto diskSample = squareToUniformDisk(sample);
    auto z = sqrt(1.0f - diskSample.squaredNorm());
    return Vector3f{diskSample.x(), diskSample.y(), z};
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    return abs(1.0f - v.norm()) < Epsilon && v.z() >= 0.0f ? INV_PI * v.z() : 0.0f;
}


Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    auto phi = 2.0f * M_PI * sample.x();
    auto theta = atanf(sqrt(-powf(alpha, 2) * log(1.0f - sample.y())));
    auto sinTheta = sin(theta);
    return Vector3f{cos(phi) * sinTheta, sin(phi) * sinTheta, cos(theta)};
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    auto alpha2 = powf(alpha, 2);
    return abs(1.0f - m.norm()) < Epsilon && m.z() >= 0.0f ?
        (expf(-powf(tan(acos(m.z())), 2) / alpha2)) / (M_PI * alpha2 * powf(m.z(), 3)) : 0.0f;
}


Vector3f Warp::squareToUniformTriangle(const Point2f &sample) {
    float su1 = sqrtf(sample.x());
    float u = 1.f - su1, v = sample.y() * su1;
    return Vector3f(u,v,1.f-u-v);
}

Vector3f Warp::squareToGTR1(const Point2f &sample, float alpha) {
    //Similar to Beckmann, Disney GTR (3)
    auto phi = 2.0f * M_PI * sample.x();
    auto theta = 0.f;
    if (alpha < 1.f) {
        auto a2 = alpha * alpha;
        theta = acosf(sqrt((1 - powf(a2, 1.f - sample.y())) / (1.f - a2)));
    }
    return Vector3f(cos(phi)*sin(theta), sin(phi)*sin(theta), cos(theta));
}

float Warp::squareToGTR1Pdf(const Vector3f &m, float alpha) {
    //Similar to Beckmann, Disney GTR (1)
    if (abs(1.f - m.norm()) > Epsilon || m.z() < 0.f)return 0.f;

    if (alpha >= 1.f)return INV_PI;

    auto cosTheta = m.z();
    auto a2 = alpha * alpha;
    auto term = 1.f + (a2 - 1.f) * cosTheta * cosTheta;
    return (a2 - 1)* cosTheta / (M_PI * log(a2) * term);
}

Vector3f Warp::squareToGTR2(const Point2f &sample, float alpha) {
    //Similar to Beckmann, Disney GTR (9)
    float phi = 2 * M_PI * sample.x();
    float a2 = alpha * alpha;
    float theta = acosf(sqrtf((1.f - sample.y()) / (1.f + (a2 - 1.f) * sample.y())));
    return Vector3f(cos(phi)*sin(theta), sin(phi)*sin(theta), cos(theta));
}

float Warp::squareToGTR2Pdf(const Vector3f &m, float alpha) {
    //Similar to Beckmann, Disney GTR (8)
    if(abs(1.f - m.norm()) > Epsilon || m.z() < 0.0f)return 0.f;

    auto cosTheta = m.z();
    float a2 = alpha * alpha;
    float term = 1.f + (a2 - 1.f) * cosTheta * cosTheta;
    return a2*cosTheta / (M_PI * powf(term,2));
}

NORI_NAMESPACE_END
