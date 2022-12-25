#include <nori/emitter.h>
#include <nori/scene.h>
#include <nori/bitmap.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN
using namespace std;
using namespace Eigen;
//https://github.com/yuqinghuang01/nori-report
//https://github.com/smeno004/CGAS2022-Project-Report
class Environment : public Emitter {
public:
    Environment(const PropertyList &props) {
        m_mapPath = props.getString("filename");
        m_envBitMap = Bitmap(m_mapPath);
        m_worldRadius = props.getFloat("radius");
    }  

    VectorXf computeCDF(VectorXf PDFVec){
        auto n = m_envBitMap.rows();
        auto res = VectorXf(n+1);
        res(0) = 0;

        for(int i = 1; i < n; i++){
            res(i) = res(i-1) + PDFVec(i-1);
        }
        res(n) = 1.f;

        return res;
    } 

    void buildIntensity(){
        auto nRow = m_envBitMap.rows();
        auto nCol = m_envBitMap.cols();
        for(int i = 0; i < nRow; i++){
            for(int j = 0; j < nCol; j++){
                Color3f val = m_envBitMap(i,j).getLuminance();
                float r = val.x();
                float g = val.y();
                float b = val.z();
                intensity(i,j) = ((r+r+b+g+g+g) / 6.f) * (sin(i) / m_envBitMap.rows()); //MAL SCHAUEN
            }
        }


        // COMPUTE THE PDF BY SUMMING ROWS AND DIVIDING BY TOTAL
        rowPDF = VectorXf(nRow);
        for(int i = 0; i < nRow; i++){
            rowPDF(i) = intensity.row(i).sum();
        }

        rowPDF /= rowPDF.sum();


        //COMPUTE THE CDF NOW
        rowCDF = computeCDF(rowPDF);

        condPDF = MatrixXf(nRow,nCol);
        condCDF = MatrixXf(nRow, nCol+1);
        
        for(int i = 0; i < nRow; i++){
            condPDF.row(i) = intensity.row(i) / intensity.row(i).sum();

            //CALCULATE CDF NOW WITH THE SAME FORMULA AS ABOVE
            auto intermediate = computeCDF(condPDF.row(i).transpose());
            condCDF.row(i) = intermediate.transpose();
        }


    }

    virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const {
       return 0;
       //From PBRT BOOK --> NEED SampleContinuous Code
    }

    virtual Color3f eval(const EmitterQueryRecord &lRec) const {
        auto normal = lRec.wi;

        //From Sphere.cpp 
        auto u = 0.5f + std::atan2(normal.y(), normal.x()) / (2.0f * M_PI);
        auto v = 0.5f + std::asin(normal.z()) / M_PI;
        

        return intensity(u,v);
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const {
       auto normal = lRec.wi;

        //From Sphere.cpp 
        auto u = 0.5f + std::atan2(normal.y(), normal.x()) / (2.0f * M_PI);
        auto v = 0.5f + std::asin(normal.z()) / M_PI;

        u = int(round(u));
        v = int(round(v));

        return condPDF(u,v) * rowPDF(u);
    }

    std::string toString() const {
        return "";
    }

private:
    MatrixXf intensity;
    string m_mapPath;
    Bitmap m_envBitMap;
    float m_worldRadius;

    VectorXf rowPDF;
    VectorXf rowCDF;
    MatrixXf condPDF;
    MatrixXf condCDF;
};

NORI_REGISTER_CLASS(Environment, "environment");
NORI_NAMESPACE_END
