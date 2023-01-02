#include <nori/phasefunction.h>
#include <nori/warp.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

class Isotropic : public PhaseFunction {
public:
    Isotropic(const PropertyList &props) {}

    float sample(Vector3f &wo, const Point2f &sample) const override {
        wo = Warp::squareToUniformSphere(sample);
        return INV_FOURPI;
    }

    std::string toString() const override {
        return tfm::format("Isotropic");
    }
};


class HenyeyGreenstein : public PhaseFunction {
public:
    HenyeyGreenstein(const PropertyList &props) {
        m_g = props.getFloat("g", 0.0f);
    }

    float sample(Vector3f &wo, const Point2f &sample) const override {
        float g_sqr = m_g * m_g;

        float cos_theta;
        if (abs(m_g) < Epsilon) {
            cos_theta = 1.0f - 2.0f * sample.x();
        } else {
            float tmp = (1.0f - g_sqr) / (1.0f - m_g + 2.0f * m_g * sample.x());
            cos_theta = -(1.0f + g_sqr - tmp * tmp) / abs(2.0f * m_g);
        }

        float sin_theta = sqrt(std::max(0.0f, 1.0f - cos_theta * cos_theta));
        float phi = 2.0f * M_PI * sample.y();

        wo = Vector3f(cos(phi) * sin_theta, sin(phi) * sin_theta, cos_theta);
        return INV_FOURPI * (1.0f - g_sqr) / powf(1.0f + g_sqr - 2.0f * m_g * cos_theta, 1.5f);
    }

    std::string toString() const override {
        return tfm::format( "HenyeyGreenstein[ g=%s ]", m_g);
    }

private:
    float m_g;
};

NORI_REGISTER_CLASS(Isotropic, "isotropic");
NORI_REGISTER_CLASS(HenyeyGreenstein, "hg");
NORI_NAMESPACE_END