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
        buildIntensity();
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

        //Init the intensity matrix otherwise core dump
        intensity = MatrixXf(nRow,nCol);

        for(int i = 0; i < nRow; i++){
            for(int j = 0; j < nCol; j++){
                float val = m_envBitMap(i,j).getLuminance();

                float sinTheta = sin(M_PI * i / (nRow));
                intensity(i,j) = val * sinTheta; 
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

    static int sampleDiscrete(const VectorXf &cdf,const float &sample) {
        int n = cdf.size();
        int a = 0;
        int b = n - 1;
        int index = -1;
        while (a <= b) {
            int m = (a + b + 1) / 2;
            if (cdf(m) <= sample) {
                index = m;
                a = m + 1;
            }
            else {
                b = m - 1;
            }
        }
        return index;
    }

    virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const {
  
        auto nRow = m_envBitMap.rows()-1;
        auto nCol = m_envBitMap.cols()-1;

        //From PBRT BOOK --> use SampleDiscrete instead of SampleContinuous

        auto i = sampleDiscrete(rowCDF, sample.x());
        auto j = sampleDiscrete(condCDF.row(i).transpose(), sample.y());
        

        auto theta = M_PI * i / nRow;
        auto phi = 2 * M_PI * j / nCol;

        lRec.wi = sphericalDirection(theta, phi);
        auto pdfValue = pdf(lRec);

        //Avoid division by zero
        if (pdfValue == 0)return 0;

        auto Jacobian = nCol *nRow / (2 * M_PI * M_PI * Frame::sinTheta(lRec.wi));
        return eval(lRec) / (Jacobian * pdfValue);       
        
    }

    virtual Color3f eval(const EmitterQueryRecord &lRec) const {
        auto normal = sphericalCoordinates(lRec.wi);

        //Use row-1 to avoid IndexOutOfBounds
        auto nRow = m_envBitMap.rows()-1;
        auto nCol = m_envBitMap.cols()-1;
        
        //Not the same conversion as in sphere.cpp
        auto x = normal.x() * nRow * INV_PI;
        auto y = normal.y() * 0.5f * nCol * INV_PI;

        int u = int(round(x));
        int v = int(round(y));

        //Edge case avoiding. Use clamp function
        /*
        auto u1 = clamp(u+1,0,nRow);
        auto u2 = clamp(u-1,0,nRow);
        auto v1 = clamp(v+1,0,nCol);
        auto v2 = clamp(v-1,0,nCol);
        */
        //take average over neighbors = BLUR
        //auto avgSurrounding = (m_envBitMap(u1,v) +  m_envBitMap(u2,v) +  m_envBitMap(u,v1) +  m_envBitMap(u,v2)) / 4.f;
        return m_envBitMap(u,v);
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const {
        auto normal = sphericalCoordinates(lRec.wi);
        auto nRow = m_envBitMap.rows()-1;
        auto nCol = m_envBitMap.cols()-1;

        //From Sphere.cpp 
        auto x = normal.x() * nRow * INV_PI;
        auto y = normal.y() * 0.5f * nCol * INV_PI;

        int u = int(round(x));
        int v = int(round(y));


        return condPDF(u,v) * rowPDF(u);  
        
    }

    std::string toString() const {
        return tfm::format(
                "Environment[\n"
                "  filename = %s,\n"
                "]",
                m_mapPath
            );
    }

private:
    MatrixXf intensity;
    string m_mapPath;
    Bitmap m_envBitMap;

    VectorXf rowPDF;
    VectorXf rowCDF;
    MatrixXf condPDF;
    MatrixXf condCDF;
};

NORI_REGISTER_CLASS(Environment, "environment");
NORI_NAMESPACE_END
