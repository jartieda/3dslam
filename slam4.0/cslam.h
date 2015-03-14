#ifndef CSLAM_H
#define CSLAM_H
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <fstream>
#include "xmlParser.h"
#include <string>
#include <vector>
#include "surf.h"

#include <cstdio>
#define MEAS_NOISE_COV 8

class CSlam
{
    public:
        CSlam(XMLNode* n);
        virtual ~CSlam();
        void run(IplImage* img);
        void predict();
        void correct();
        void test();
        XMLNode *xMainNode;
        CvMat* String2CvMat(char *s,int cols, int rows) ;
        void StringSplit(std::string str, std::string delim, std::vector<std::string> *results);
        void transMat(CvMat* o_mat, CvMat* d_mat);
        void MemMat2WorkMat();
        void matchHarris(IplImage *img);
        void matchFile(IplImage *img);
        void matchSurf(IplImage *img);
        void addHarris(IplImage* img);
        void addFile(IplImage* img);
        void addSurf(IplImage* img);
        int Detector;
        int numVisibleMin;
        int visNum;
        void projectAllPoints(CvMat *state);
        void InverseDepth2Depth(double x, double y, double z, double theta, double phi, double rho, double *outx, double*outy, double *outz);
        void ProjectPoints3(  CvPoint3D64f* M,
                            CvMat* state,
                             CvMat* A,
                            CvMat* dist_coeffs,
                            CvPoint2D64f* m, CvMat* dpdr,
                            CvMat* dpdt, CvMat* dpdf,
                            CvMat* dpdc, CvMat* dpdk ,CvMat* dpdw);
        void Draw(IplImage *framecopy);
        void PrintKalman();

        //Variables Harris
        int searchZoneSize;///<size of the search window for Harris alg. ex: 50 pix
        int patternSize;   ///< size of the pattern to be matched for Harris alg. ex: 10 pix
        float minCorrVal;  ///< minimum correlation value for a match to be accepted ex: 0.9
        float MinBorderGreyDist; ///< maximum distance between a mathc in grey image and one in border image for a match to be accepted: ex 20 to deactivate use 50
        int cornercount;   ///< maximum corners to be added at a time ex:30
        float quality_level; ///< don't add corners with egigenv. below this threshold ex: 0.1
        float min_distance; ///< minimum distance between corners. ex 30
//    protected:
        double avalue; ///<postion variation model for the construcion of Pn matrix in correct fn
        double alpha; ///<angle variation model for the construction of Pn matrix in correct() fn
        int SD;///<State Dimension
        int MD;///<Measurement Dimension
        int CD;///<Control Dimension
        int modelSD;///<State Dimension
        int modelMD;///<Measurement Dimension
        int modelCD;///<Control Dimension
        CvMat* Xp; ///< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
        CvMat* X;          ///< corrected state (x(k)):  x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
        CvMat* A;   ///< state transition matrix (A)
        CvMat* B;      ///< control matrix (B) (it is not used if there is no control)
        CvMat* H;  ///< measurement matrix (H) visible
        CvMat* HFull; ///< measurment matrix (HFull) visible and not visible
        CvMat* Q;   ///< process noise covariance matrix (Q)
        CvMat* R; ///< measurement noise covariance matrix (R)
        CvMat* Pp;       ///< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
        CvMat* K;                ///< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)*/
        CvMat* P;      ///< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k) */
        CvMat* control; ///<control vector
        //CvMat* measurementVisible; ///<measurement done by the matching/tracking algorithm
        std::vector<float>prediction; ///<measurement predicted by the model
        std::vector<float>measurement;///<measurement done by the matching/tracking algorithm
        std::vector<int> visible; ///<vector storing the visibility of each point

        CvMat* XpMem; ///< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
        CvMat* XMem;          ///< corrected state (x(k)):  x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
        CvMat* AMem;   ///< state transition matrix (A)
        CvMat* BMem;      ///< control matrix (B) (it is not used if there is no control)
        CvMat* QMem;   ///< process noise covariance matrix (Q)
        CvMat* RMem; ///< measurement noise covariance matrix (R)
        CvMat* HMem;  ///< measurement matrix (H)
        CvMat* HFullMem;  ///< measurement matrix (H)
        CvMat* PpMem;       ///< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
        CvMat* KMem;                /// <Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)*/
        CvMat* PMem;      ///< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k) */
        CvMat* predictionMem; ///<measurement predicted by the model
        CvMat* measurementMem; ///<measurement done

        CvMat* temp1;               /* temporary matrices */
        CvMat* temp2;
        CvMat* temp3;
        CvMat* temp4;
        CvMat* temp5;

        CvMat* temp1Mem;               /* temporary matrices */
        CvMat* temp2Mem;
        CvMat* temp3Mem;
        CvMat* temp4Mem;
        CvMat* temp5Mem;
        /** surf tracker vbles **/
        IplImage *fImg; ///<float point image por temporary calcs
        IplImage *old_img; ///<previous image
        CSurf surf; ///<surf class
        int levels; ///<level number in surf
        CvSeq* feat; ///<Feature sequence as extracted from CSurf class
        CvMemStorage* storage; ///< memory storage for CvSeq* feat.
        CvSeq* feat_old; ///<Feature sequence as extracted from CSurf class
        CvMemStorage* storage_old; ///< memory storage for CvSeq* feat.
        std::vector<int *> keys; ///< feature keys from extracted points in surf
        /** pinhole model vbles **/
        CvMat* IntrinsicParam;
        CvMat* DistortionParam;
        void InverseParam(CvMat** h,CvPoint pto);
        float depth; ///< feature initialization depth
        //Data Out vbles
        void printMat(CvMat *m);
        CvFont font;
        double hScale;
        double vScale;
        int    lineWidth;
        char strID[10];
        void CopyMat(CvMat* o, CvMat* d, int row,int col);
        float ic_test(int pointNum, float measx, float measy);
        int first;
        void AddPointToCovMatrix(double x, double y);
        std::ofstream DispFile;
        void Disp_out(IplImage *framecopy);
        void testStereo(IplImage *img, IplImage *old_img, double minshift, double maxshift);
        void testOpticalFlow(IplImage* img1, IplImage* img2, double minshiftX, double maxshiftX, double minshiftY, double maxshiftY);
        void JCBB( int level, int MaxRama);
        void JCBB_incremental(int level,int MaxRama,
                 CvMat *C, CvMat* Cinv, CvMat* H, CvMat* innov);
        bool joint_compatibility(std::vector<int> visible, std::vector<float> Meas);
        bool joint_compatibility_incremental(std::vector<int> visible_, std::vector<float> Meas,
                                            CvMat *C_1, CvMat* Cinv_1, CvMat* H_1, CvMat* innov_1,
                                            CvMat **C, CvMat** Cinv, CvMat** H, CvMat** innov);
        void QuadForm(CvMat *A, CvMat *vect, CvMat* resultado,CvMat *B=NULL);
        std::vector<int> BestH;
        std::vector<int> H_;
        int pairingsBest;
        int maxLevels;
        void fillInvParamDeriv(CvMat* dpdw, CvMat* dpdW, float theta, float phi, float rho);
        void v2q(CvMat* v, CvMat* q);
        void quatMult(CvMat* q, CvMat* p,CvMat *dest);
        void dq3dq2(CvMat* q2, CvMat *der);
        void dq3dq1(CvMat* q2, CvMat *der);
        void dv2qdv(CvMat *v, CvMat *dqdv);
        void getTransRot(CvMat *state, CvMat *t, CvMat *q, CvMat *Rot);
        void dR_by_dq(CvMat *dRdr,CvMat *r_vec);
        float randomVector(float max,float min);
        void NormalizeQuatCov(CvMat *state, CvMat *_P);
        void Cholesky(CvMat *in, CvMat *out);
        CvMat* Simul_State;
    private:
        void ReserveMemory();
};

#endif // CSLAM_H
