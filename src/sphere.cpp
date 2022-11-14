/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Sphere : public Shape {
public:
    Sphere(const PropertyList & propList) {
        m_position = propList.getPoint3("center", Point3f());
        m_radius = propList.getFloat("radius", 1.f);

        m_bbox.expandBy(m_position - Vector3f(m_radius));
        m_bbox.expandBy(m_position + Vector3f(m_radius));
    }

    virtual BoundingBox3f getBoundingBox(uint32_t index) const override { return m_bbox; }

    virtual Point3f getCentroid(uint32_t index) const override { return m_position; }

    virtual bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const override {
        auto a = ray.d.squaredNorm();
        auto b = 2 * (ray.d.dot(ray.o - m_position));
        auto c = (ray.o - m_position).squaredNorm() - powf(m_radius, 2);

        auto d = powf(b, 2) - 4 * a * c;

        float t0, t1;
        if (d > 0) {
            auto a2 = 2 * a;
            t0 = (-b - sqrt(d)) / a2;
            t1 = (-b + sqrt(d)) / a2;
        } 
        else if (d == 0) {
            t0 = -b / (2 * a);
            t1 = t0;
        } 
        else {  // d < 0 --> no root --> ray does not intersect sphere
            return false;
        }

        // Intersection would be in negative direction
        if (t0 < 0 && t1 < 0) {
            return false;
        }

        if (ray.mint <= t0 && t0 <= ray.maxt) {
            t = t0;
            return true;
        }
        else if (ray.mint <= t1 && t1 <= ray.maxt) {
            t = t1;
            return true;
        }

        return false;
    }

    virtual void setHitInformation(uint32_t index, const Ray3f &ray, Intersection &its) const override {
        its.p = ray.o + ray.d * its.t;

        Vector3f normal = (its.p - m_position).normalized();
        its.geoFrame = Frame(normal);
        its.shFrame = its.geoFrame;

        its.uv = sphericalCoordinates(normal);

        /**
         * Because nori uses (acos(z), atan2(y, x)) as spherical coordinates
         * (usually it would be (0.5 + atan2(x, z)/2pi,  0.5 + asin(y)/pi) as seen in https://en.wikipedia.org/wiki/UV_mapping),
         * we need to scale as follows
         *      uv.x = uv.x / (2 * pi) + 0.5
         *      uv.y = uv.y / pi
         * */     
        its.uv.x() = its.uv.x() / (2.0f * M_PI) + 0.5f;
        its.uv.y() = its.uv.y() / M_PI;
    }

    virtual void sampleSurface(ShapeQueryRecord & sRec, const Point2f & sample) const override {
        Vector3f q = Warp::squareToUniformSphere(sample);
        sRec.p = m_position + m_radius * q;
        sRec.n = q;
        sRec.pdf = std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
    }
    virtual float pdfSurface(const ShapeQueryRecord & sRec) const override {
        return std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
    }


    virtual std::string toString() const override {
        return tfm::format(
                "Sphere[\n"
                "  center = %s,\n"
                "  radius = %f,\n"
                "  bsdf = %s,\n"
                "  emitter = %s\n"
                "]",
                m_position.toString(),
                m_radius,
                m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
                m_emitter ? indent(m_emitter->toString()) : std::string("null"));
    }

protected:
    Point3f m_position;
    float m_radius;
};

NORI_REGISTER_CLASS(Sphere, "sphere");
NORI_NAMESPACE_END
