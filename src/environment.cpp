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
  
        int row = m_envBitMap.rows();
        int col = m_envBitMap.cols();
        auto i = sampleDiscrete(rowCDF, sample.x());
        auto j = sampleDiscrete(condCDF.row(i).transpose(), sample.y());
        //From PBRT BOOK --> NEED SampleContinuous Code

        auto theta = M_PI * i / (row - 1);
        auto phi = 2 * M_PI * j / (col - 1);

        lRec.wi = sphericalDirection(theta, phi);
        Ray3f ray(lRec.ref, lRec.wi, Epsilon, std::numeric_limits<double>::infinity());

                
        auto pdfValue = pdf(lRec);
        if (pdfValue == 0) {
            return 0;
        }

        auto J = (col - 1) * (row - 1) / (2 * M_PI * M_PI * Frame::sinTheta(lRec.wi));
        return eval(lRec) / (J * pdfValue);       
        
    }

    virtual Color3f eval(const EmitterQueryRecord &lRec) const {
        auto normal = sphericalCoordinates(lRec.wi);
        auto nRow = m_envBitMap.rows()-1;
        auto nCol = m_envBitMap.cols()-1;
        //From Sphere.cpp 
        auto x = normal.x() * nRow * INV_PI;
        auto y = normal.y() * 0.5f * nCol * INV_PI;

        int u = int(round(x));
        int v = int(round(y));

        
        //return Color3f(intensity(u,v));
        return m_envBitMap(u,v);
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const {
        auto normal = sphericalCoordinates(lRec.wi);
        auto nRow = m_envBitMap.rows();
        auto nCol = m_envBitMap.cols();
        //From Sphere.cpp 
        auto x = normal.x() * nRow * INV_PI;
        auto y = normal.y() * 0.5f * nCol * INV_PI;

        int u = int(round(x));
        int v = int(round(y));


        return condPDF(u,v) * rowPDF(u);  
        
    }

    std::string toString() const {
        return tfm::format("Environment[]");
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
