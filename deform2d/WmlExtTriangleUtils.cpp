#include "WmlExtTriangleUtils.h"

#include <limits>

using namespace Wml;

template <class Real>
void Wml::BarycentricCoords( const Vector3<Real> & vTriVtx1, 
							 const Vector3<Real> & vTriVtx2,
							 const Vector3<Real> & vTriVtx3,
							 const Vector3<Real> & vVertex,
							 Real & fBary1, Real & fBary2, Real & fBary3 )
{

	Wml::Vector3<Real> kV02 = vTriVtx1 - vTriVtx3;
    Wml::Vector3<Real> kV12 = vTriVtx2 - vTriVtx3;
    Wml::Vector3<Real> kPV2 = vVertex - vTriVtx3;

    Real fM00 = kV02.Dot(kV02);
    Real fM01 = kV02.Dot(kV12);
    Real fM11 = kV12.Dot(kV12);
    Real fR0 = kV02.Dot(kPV2);
    Real fR1 = kV12.Dot(kPV2);
    Real fDet = fM00*fM11 - fM01*fM01;
//    ASSERT( Wml::Math<Real>::FAbs(fDet) > (Real)0.0 );
    Real fInvDet = ((Real)1.0)/fDet;

    fBary1 = (fM11*fR0 - fM01*fR1)*fInvDet;
    fBary2 = (fM00*fR1 - fM01*fR0)*fInvDet;
    fBary3 = (Real)1.0 - fBary1 - fBary2;
}

template <class Real>
void Wml::BarycentricCoords( const Vector2<Real> & vTriVtx1, 
							 const Vector2<Real> & vTriVtx2,
							 const Vector2<Real> & vTriVtx3,
							 const Vector2<Real> & vVertex,
							 Real & fBary1, Real & fBary2, Real & fBary3 )
{

	Wml::Vector2<Real> kV02 = vTriVtx1 - vTriVtx3;
    Wml::Vector2<Real> kV12 = vTriVtx2 - vTriVtx3;
    Wml::Vector2<Real> kPV2 = vVertex - vTriVtx3;

    Real fM00 = kV02.Dot(kV02);
    Real fM01 = kV02.Dot(kV12);
    Real fM11 = kV12.Dot(kV12);
    Real fR0 = kV02.Dot(kPV2);
    Real fR1 = kV12.Dot(kPV2);
    Real fDet = fM00*fM11 - fM01*fM01;
//    ASSERT( Wml::Math<Real>::FAbs(fDet) > (Real)0.0 );
    Real fInvDet = ((Real)1.0)/fDet;

    fBary1 = (fM11*fR0 - fM01*fR1)*fInvDet;
    fBary2 = (fM00*fR1 - fM01*fR0)*fInvDet;
    fBary3 = (Real)1.0 - fBary1 - fBary2;
}

template <int N, class Real>
void Wml::Scale( Vector<N,Real> & vTriV0,
				 Vector<N,Real> & vTriV1,
				 Vector<N,Real> & vTriV2,
				 Real fScale )
{
	// find center of mass
	Wml::Vector<N,Real> vCentroid( vTriV0 + vTriV1 + vTriV2 );
	vCentroid *= (Real)1.0 / (Real)3.0;

	// convert to vectors, scale and restore
	vTriV0 -= vCentroid;	vTriV0 *= fScale;	vTriV0 += vCentroid;
	vTriV1 -= vCentroid;	vTriV1 *= fScale;	vTriV1 += vCentroid;
	vTriV2 -= vCentroid;	vTriV2 *= fScale;	vTriV2 += vCentroid;
}

namespace Wml
{
template  void BarycentricCoords( const Vector3<float> & TriVtx1, const Vector3<float> & TriVtx2,
											 const Vector3<float> & TriVtx3, const Vector3<float> & vVertex,
											 float & fWeight1, float & fWeight2, float & fWeight3 );
template  void BarycentricCoords( const Vector3<double> & TriVtx1, const Vector3<double> & TriVtx2,
											 const Vector3<double> & TriVtx3, const Vector3<double> & vVertex,
											 double & fWeight1, double & fWeight2, double & fWeight3 );
template  void BarycentricCoords( const Vector2<float> & TriVtx1, const Vector2<float> & TriVtx2,
											 const Vector2<float> & TriVtx3, const Vector2<float> & vVertex,
											 float & fWeight1, float & fWeight2, float & fWeight3 );
template  void BarycentricCoords( const Vector2<double> & TriVtx1, const Vector2<double> & TriVtx2,
											 const Vector2<double> & TriVtx3, const Vector2<double> & vVertex,
											 double & fWeight1, double & fWeight2, double & fWeight3 );

template  void Scale(  Vector<2,float> & TriVtx1,  Vector<2,float> & TriVtx2, Vector<2,float> & TriVtx3, float fScale );
template  void Scale(  Vector<2,double> & TriVtx1,  Vector<2,double> & TriVtx2, Vector<2,double> & TriVtx3, double fScale );
template  void Scale(  Vector<3,float> & TriVtx1,  Vector<3,float> & TriVtx2, Vector<3,float> & TriVtx3, float fScale );
template  void Scale(  Vector<3,double> & TriVtx1,  Vector<3,double> & TriVtx2, Vector<3,double> & TriVtx3, double fScale );
}
