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

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/shape.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter {
public:
    AreaEmitter(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }

    virtual std::string toString() const override {
        return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                "]",
                m_radiance.toString());
    }

    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        return lRec.n.dot(lRec.wi) < 0 ? m_radiance : Color3f{0.0f};
    }

    virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        ShapeQueryRecord shapeRecord{lRec.ref};
        m_shape->sampleSurface(shapeRecord, sample);

        lRec.p = shapeRecord.p;
        lRec.n = shapeRecord.n;
        
        auto shadowRayDir = (lRec.p - lRec.ref);
        lRec.wi = shadowRayDir.normalized();
        lRec.shadowRay = Ray3f{lRec.ref, lRec.wi, Epsilon, shadowRayDir.norm() - Epsilon};

        auto p = pdf(lRec);
        lRec.pdf = p;

        if (p <= 0) return Color3f{0.0f};

        return eval(lRec) / p;
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        auto cosTheta = lRec.n.dot(-lRec.wi);

        if (cosTheta <= 0) return 0.0f;

        ShapeQueryRecord shapeRecord{lRec.ref, lRec.p};
        auto pdfSurface = m_shape->pdfSurface(shapeRecord);
        
        // Slides Direct Illumination II, p. 22
        return (lRec.ref - lRec.p).squaredNorm() / cosTheta * pdfSurface;  // p_omega = d^2 / cos(theta) * p_A
    }


    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
        ShapeQueryRecord shapeRecord;
        m_shape->sampleSurface(shapeRecord, sample1);

        auto direction = Frame(shapeRecord.n).toWorld(Warp::squareToCosineHemisphere(sample2));
        ray = Ray3f{shapeRecord.p, direction};

        if (shapeRecord.pdf <= 0) {
            return Color3f{0.0f};
        }

        EmitterQueryRecord emitterRecord{shapeRecord.p + direction, shapeRecord.p, shapeRecord.n};
        return M_PI / shapeRecord.pdf * eval(emitterRecord);     // pi * area * Le = pi * 1/pdf * Le
    }


protected:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END