#if !defined(__PHASE_H__)
#define __PHASE_H__

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

class PhaseFunction : public NoriObject {
public:
    virtual float sample(Vector3f &wo, const Point2f &sample) const = 0;

    virtual EClassType getClassType() const { return EPhaseFunction; }
};

NORI_NAMESPACE_END
#endif 