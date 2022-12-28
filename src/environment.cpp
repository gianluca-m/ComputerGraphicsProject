#include <nori/emitter.h>
#include <nori/scene.h>
#include <nori/bitmap.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN
using namespace std;
using namespace Eigen;

class Environment : public Emitter {
public:
    Environment(const PropertyList &props) {
        m_mapPath = props.getString("filename");
        m_interpolate = props.getBoolean("interpolate", true);
        m_envBitMap = Bitmap(m_mapPath);
        buildIntensity();
    }  

    VectorXf computeCDF(VectorXf PDFVec) {
        auto n = m_envBitMap.rows();
        auto res = VectorXf(n+1);
        res(0) = 0;

        for(int i = 1; i < n; i++) {
            res(i) = res(i - 1) + PDFVec(i - 1);
        }
        res(n) = 1.f;

        return res;
    } 

    void buildIntensity() {
        auto nRow = m_envBitMap.rows();
        auto nCol = m_envBitMap.cols();

        // Init the intensity matrix otherwise core dump
        intensity = MatrixXf(nRow,nCol);

        for(int i = 0; i < nRow; i++) {
            for(int j = 0; j < nCol; j++) {
                float val = m_envBitMap(i, j).getLuminance();

                float sinTheta = sin(M_PI * i / (nRow));
                intensity(i, j) = val * sinTheta; 
            }
        }

        // COMPUTE THE PDF BY SUMMING ROWS AND DIVIDING BY TOTAL
        rowPDF = VectorXf(nRow);
        for(int i = 0; i < nRow; i++) {
            rowPDF(i) = intensity.row(i).sum();
        }

        rowPDF /= rowPDF.sum();

        // COMPUTE THE CDF NOW
        rowCDF = computeCDF(rowPDF);

        condPDF = MatrixXf(nRow, nCol);
        condCDF = MatrixXf(nRow, nCol + 1);
        
        for(int i = 0; i < nRow; i++){
            condPDF.row(i) = intensity.row(i) / intensity.row(i).sum();

            // CALCULATE CDF NOW WITH THE SAME FORMULA AS ABOVE
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

    Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const override {
        auto nRow = m_envBitMap.rows() - 1;
        auto nCol = m_envBitMap.cols() - 1;

        // From PBRT BOOK --> use SampleDiscrete instead of SampleContinuous
        auto i = sampleDiscrete(rowCDF, sample.x());
        auto j = sampleDiscrete(condCDF.row(i).transpose(), sample.y());
        
        auto theta = M_PI * i / nRow;
        auto phi = 2 * M_PI * j / nCol;

        lRec.wi = sphericalDirection(theta, phi);
        auto pdfValue = pdf(lRec);

        // Avoid division by zero
        if (pdfValue == 0) return 0;

        auto Jacobian = nCol *nRow / (2 * M_PI * M_PI * Frame::sinTheta(lRec.wi));
        return eval(lRec) / (Jacobian * pdfValue);       
    }

    Color3f eval(const EmitterQueryRecord &lRec) const override {
        auto normal = sphericalCoordinates(lRec.wi);

        // Use row-1 to avoid IndexOutOfBounds
        int nRow = m_envBitMap.rows() - 1;
        int nCol = m_envBitMap.cols() - 1;
        
        // Not the same conversion as in sphere.cpp
        auto x = normal.x() * nRow * INV_PI;
        auto y = normal.y() * 0.5f * nCol * INV_PI;

        if(!m_interpolate){
            int u = int(round(x));
            int v = int(round(y));

            return m_envBitMap(u, v);
        }

        // https://helloacm.com/cc-function-to-compute-the-bilinear-interpolation/
        // ADAPTED FROM THIS WEBSITE
        int topX = int(ceil(x));
        int topY = int(ceil(y));
        int bottomY = int(floor(y));
        int bottomX = int(floor(x));

        auto x1 = clamp(bottomX, 0, nRow);
        auto x2 = clamp(topX, 0, nRow);
        auto y1 = clamp(bottomY, 0, nCol);
        auto y2 = clamp(topY, 0, nCol);

        auto q11 = m_envBitMap(x1, y1);
        auto q12 = m_envBitMap(x1, y2);
        auto q21 = m_envBitMap(x2, y1);
        auto q22 = m_envBitMap(x2, y2);

        auto xDiff = x - x1;
        auto yDiff = y - y1;

        // Edge Scenario where clamp returns the same value
        if(x1 == x2 || y2 == y1) return 0;

        return lerpColor(yDiff, lerpColor(xDiff, q11, q21), lerpColor(xDiff, q12, q22));
    }

    float pdf(const EmitterQueryRecord &lRec) const override {
        auto normal = sphericalCoordinates(lRec.wi);
        auto nRow = m_envBitMap.rows() - 1;
        auto nCol = m_envBitMap.cols() - 1;

        // From Sphere.cpp 
        auto x = normal.x() * nRow * INV_PI;
        auto y = normal.y() * 0.5f * nCol * INV_PI;
        
        int u = int(round(x));
        int v = int(round(y));

        return condPDF(u, v) * rowPDF(u);  
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
    bool m_interpolate;

    VectorXf rowPDF;
    VectorXf rowCDF;
    MatrixXf condPDF;
    MatrixXf condCDF;

    Color3f lerpColor(float t, const Color3f v1, const Color3f v2) const {
        return (1 - t) * v1 + t * v2;
    }
};

NORI_REGISTER_CLASS(Environment, "environment");
NORI_NAMESPACE_END
