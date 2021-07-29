#ifndef __RMS_WML_EXT_TRIANGLE_UTIL_H__
#define __RMS_WML_EXT_VECTOR_UTIL_H__

#include <WmlVector2.h>
#include <WmlVector3.h>

namespace Wml
{
	template <class Real>
	void BarycentricCoords( const Vector3<Real> & vTriVtx1, 
										const Vector3<Real> & vTriVtx2,
										const Vector3<Real> & vTriVtx3,
										const Vector3<Real> & vVertex,
										Real & fBary1, Real & fBary2, Real & fBary3 );

	template <class Real>
	void BarycentricCoords( const Vector2<Real> & vTriVtx1, 
										const Vector2<Real> & vTriVtx2,
										const Vector2<Real> & vTriVtx3,
										const Vector2<Real> & vVertex,
										Real & fBary1, Real & fBary2, Real & fBary3 );

	template <int N, class Real>
	void Scale( Vector<N,Real> & vTriV0,
							Vector<N,Real> & vTriV1,
							Vector<N,Real> & vTriV2,
							Real fScale );
}  // end namespace Wml

#endif // __RMS_WML_EXT_TRIANGLE_UTIL_H__
