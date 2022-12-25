#include <nori/emitter.h>
#include <nori/scene.h>
#include <nori/bitmap.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN
using namespace std;
//https://github.com/yuqinghuang01/nori-report
//https://github.com/smeno004/CGAS2022-Project-Report
class Environment : public Emitter {
public:
    Environment(const PropertyList &props) {
        m_mapPath = props.getString("filename");
        m_envBitMap = Bitmap(m_mapPath);
        m_worldRadius = props.getFloat("radius");
    }   

    void buildIntensity(){
        for(int i = 0; i < m_envBitMap.rows(); i++){
            for(int j = 0; j < m_envBitMap.cols(); j++){
                Color3f val = m_envBitMap(i,j).getLuminance();
                float r = val.x();
                float g = val.y();
                float b = val.z();
                intensity(i,j) = ((r+r+b+g+g+g) / 6.f) * (sin(i) / m_envBitMap.rows());
            }
        }
    }

    virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const {
       return 0;
    }

    virtual Color3f eval(const EmitterQueryRecord &lRec) const {
        return M_PI * m_worldRadius * m_worldRadius; // * Value of Spectrum
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const {
       return 0;
    }

    std::string toString() const {
        return "";
    }

private:
    MatrixXf intensity;
    string m_mapPath;
    Bitmap m_envBitMap;
    float m_worldRadius;
};

NORI_REGISTER_CLASS(Environment, "environment");
NORI_NAMESPACE_END
