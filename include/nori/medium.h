#if !defined(__NORI_MEDIUM_H)
#define __NORI_MEDIUM_H

#include <nori/object.h>
#include <nori/bbox.h>
#include <nori/sampler.h>
#include <nori/phasefunction.h>

NORI_NAMESPACE_BEGIN

struct MediumQueryRecord {
    /// Interaction point
    Point3f p;

    /// Max Free Path distance
    float tMax;

    /// Sampling success
    bool hasInteraction = false;

    MediumQueryRecord() {};

    MediumQueryRecord(float tMax) : tMax(tMax) {};
};

class Medium : public NoriObject {
public:
    /// @brief Sample point within medium
    virtual Color3f sample(const Ray3f &ray, Sampler *sampler, MediumQueryRecord &mRec) const = 0;

    /// @brief Calculate Transmittance
    virtual Color3f Tr(const Ray3f &ray, Sampler *sampler, MediumQueryRecord &mRec) const = 0;

    virtual bool rayIntersect(const Ray3f &ray, float &nearT, float &farT) const = 0;

    PhaseFunction* getPhasefunction() const { return m_phasefunction; }

    virtual EClassType getClassType() const override { return EMedium; }

protected:
    Color3f m_sigma_a;
    Color3f m_sigma_s;
    Color3f m_sigma_t;
    Color3f m_albedo;

    float m_max_density;
    float m_density_scale;

    PhaseFunction *m_phasefunction = nullptr;
    BoundingBox3f m_bbox;
};

NORI_NAMESPACE_END
#endif