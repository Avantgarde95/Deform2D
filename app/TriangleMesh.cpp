// TriangleMesh.cpp: implementation of the TriangleMesh class.
//
//////////////////////////////////////////////////////////////////////

#include "TriangleMesh.h"
#include "LinearAlgebra.h"

#include <fstream>

using namespace rmsmesh;

// [RMS] need to export stl vector template instantiations
//template class RMSIMPLICIT_API std::allocator<float>;
//template class RMSIMPLICIT_API std::allocator<unsigned int>;
//template class RMSIMPLICIT_API std::vector<float, std::allocator<float> >;
//template class RMSIMPLICIT_API std::vector<unsigned int, std::allocator<unsigned int> >;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TriangleMesh::TriangleMesh()
  : tm_filename("default.obj"),
    tm_fileformat(OBJ_FORMAT)
{
  tm_initialize(0,0);
}


TriangleMesh::TriangleMesh(string filename,
	     unsigned int vertex_sizehint,
	     unsigned int triangle_sizehint)
  : tm_filename(filename)
{
  tm_initialize(vertex_sizehint, triangle_sizehint);

  // should read file here and determine type...
  tm_readobj();
}

TriangleMesh::~TriangleMesh()
{
}


void TriangleMesh::GetVertex( VertexID vID, Eigen::Vector3f & vVertex, Eigen::Vector3f * pNormal ) const
{
	vVertex.x() = tm_vertices[ vID * TM_VERTEX_STRIDE ];
	vVertex.y() = tm_vertices[ vID * TM_VERTEX_STRIDE + 1];
	vVertex.z() = tm_vertices[ vID * TM_VERTEX_STRIDE + 2];
	if ( pNormal ) {
		pNormal->x() = tm_normals[ vID * TM_VERTEX_STRIDE ];
		pNormal->y() = tm_normals[ vID * TM_VERTEX_STRIDE + 1];
		pNormal->z() = tm_normals[ vID * TM_VERTEX_STRIDE + 2];
	}
}


void TriangleMesh::GetTriangle( TriangleID tID, Eigen::Vector3f vTriangle[3], Eigen::Vector3f * pNormals  ) const
{
	GetVertex( tm_triangles[tID * TM_TRIANGLE_STRIDE], vTriangle[0] );
	GetVertex( tm_triangles[tID * TM_TRIANGLE_STRIDE + 1], vTriangle[1] );
	GetVertex( tm_triangles[tID * TM_TRIANGLE_STRIDE + 2], vTriangle[2] );
	if ( pNormals ) {
		GetNormal( tm_triangles[tID * TM_TRIANGLE_STRIDE], pNormals[0] );
		GetNormal( tm_triangles[tID * TM_TRIANGLE_STRIDE + 1], pNormals[1] );
		GetNormal( tm_triangles[tID * TM_TRIANGLE_STRIDE + 2], pNormals[2] );
	}
}



bool
TriangleMesh::read(const char * pFilename, FileFormat eFormat)
{
	tm_filename = std::string(pFilename);

	switch (eFormat) {
		case OBJ_FORMAT:
			return tm_readobj();
			break;
	}

	tm_fileerror = std::string("not implemented");
	return false;
}

bool
TriangleMesh::write(const char * pFilename, FileFormat eFormat)
{
	if ( pFilename != NULL )
		tm_filename = string(pFilename);


	switch (eFormat) {
		case OBJ_FORMAT:
			return tm_writeobj();
			break;

		case MESHLITE_FORMAT:
			return tm_writeMeshLite();
			break;
	}

	return false;
}



bool TriangleMesh::HasVertexTextureCoords() const
{
	return (tm_texture_coords.size() / TM_TEXTURE_STRIDE) == GetNumVertices();
}
bool TriangleMesh::HasTriangleTextureCoords() const
{
	return (tm_triTexCoords.size() / TM_TRITEXCOORD_STRIDE) == GetNumTriangles();
}




void
TriangleMesh::SetVertexData(unsigned int index, float * vertex,
	      float * normal, float * color,
	      float * texture_coord)
{
  unsigned int vi = TM_VERTEX_INDEX(index, 0);
  if(vertex != NULL)
    memcpy(GetVertexPointer() + vi, vertex, sizeof(float)*TM_VERTEX_STRIDE);
  if(normal != NULL)
    memcpy(GetNormalPointer() + vi, normal, sizeof(float)*TM_VERTEX_STRIDE);
  if(color != NULL)
    memcpy(GetColorPointer() + vi, color, sizeof(float)*TM_VERTEX_STRIDE);
  if(texture_coord != NULL)
    memcpy(GetTextureCoordsPointer() + TM_TEXTURE_INDEX(index,0),
	   texture_coord, sizeof(float)*TM_TEXTURE_STRIDE);
}


void
TriangleMesh::SetTriangleData(unsigned int index, unsigned int * triangle)
{
  if(triangle != NULL)
    memcpy(GetTrianglePointer() + TM_TRIANGLE_INDEX(index, 0),
	   triangle, sizeof(unsigned int)*TM_TRIANGLE_STRIDE);
}


void
TriangleMesh::AddVertexData(unsigned int index, float * vertex,
							float * normal, float * color,
							float * texture_coord)
{
	unsigned int vi = TM_VERTEX_INDEX(index, 0);
	if(vertex != NULL){
		if(tm_vertices.size() < vi+TM_VERTEX_STRIDE)
			tm_vertices.resize(vi+TM_VERTEX_STRIDE);
		memcpy(GetVertexPointer() + vi, vertex, sizeof(float)*TM_VERTEX_STRIDE);
	}
	if(normal != NULL){
		if(tm_normals.size() < vi+TM_VERTEX_STRIDE)
			tm_normals.resize(vi+TM_VERTEX_STRIDE);
		memcpy(GetNormalPointer() + vi, normal, sizeof(float)*TM_VERTEX_STRIDE);
	}
	if(color != NULL){
		if(tm_colors.size() < vi+TM_VERTEX_STRIDE)
			tm_colors.resize(vi+TM_VERTEX_STRIDE);
		memcpy(GetColorPointer() + vi, color, sizeof(float)*TM_VERTEX_STRIDE);
	}
	if(texture_coord != NULL){
		unsigned int ti = TM_TEXTURE_INDEX(index,0);
		if(tm_texture_coords.size() < ti+TM_TEXTURE_STRIDE)
			tm_texture_coords.resize(ti+TM_TEXTURE_STRIDE);
		memcpy(GetTextureCoordsPointer() + ti,
			texture_coord, sizeof(float)*TM_TEXTURE_STRIDE);
	}
}

void
TriangleMesh::AddTriangleData(unsigned int index, unsigned int * triangle)
{
	if(triangle != NULL){
		unsigned int ti = TM_TRIANGLE_INDEX(index, 0);
		if(tm_triangles.size() < ti+TM_TRIANGLE_STRIDE)
			tm_triangles.resize(ti+TM_TRIANGLE_STRIDE);
		memcpy(GetTrianglePointer() + ti,
			triangle, sizeof(unsigned int)*TM_TRIANGLE_STRIDE);
	}
}

void
TriangleMesh::AddTriTexCoordData( unsigned int nIndex, const float * pUV1, const float * pUV2, const float * pUV3 )
{
	unsigned int ti =  TM_TRITEXCOORD_INDEX(nIndex, 0);
	if ( tm_triTexCoords.size() < ti + TM_TRITEXCOORD_STRIDE )
		tm_triTexCoords.resize( ti + TM_TRITEXCOORD_STRIDE );
	tm_triTexCoords[ti++] = pUV1[0];
	tm_triTexCoords[ti++] = pUV1[1];
	tm_triTexCoords[ti++] = pUV2[0];
	tm_triTexCoords[ti++] = pUV2[1];
	tm_triTexCoords[ti++] = pUV3[0];
	tm_triTexCoords[ti++] = pUV3[1];
}




unsigned int
TriangleMesh::AppendVertexData(float * vertex, float * normal,
				float * color, float * texture_coord)
{
  unsigned int vi = 0;
  unsigned int ret = 0;
  if(vertex != NULL){
    vi = (unsigned int)tm_vertices.size();
    tm_vertices.resize(vi+TM_VERTEX_STRIDE);
    memcpy(GetVertexPointer() + vi, vertex, sizeof(float)*TM_VERTEX_STRIDE);
    ret = vi / TM_VERTEX_STRIDE;
  }
  if(normal != NULL){
    vi = (unsigned int)tm_normals.size();
    tm_normals.resize(vi+TM_VERTEX_STRIDE);
    memcpy(GetNormalPointer() + vi, normal, sizeof(float)*TM_VERTEX_STRIDE);
    ret = vi / TM_VERTEX_STRIDE;
  }
  if(color != NULL){
    vi = (unsigned int)tm_colors.size();
    tm_colors.resize(vi+TM_VERTEX_STRIDE);
    memcpy(GetColorPointer() + vi, color, sizeof(float)*TM_VERTEX_STRIDE);
    ret = vi / TM_VERTEX_STRIDE;
  }
  if(texture_coord != NULL){
    vi = (unsigned int)tm_texture_coords.size();
    tm_texture_coords.resize(vi+TM_TEXTURE_STRIDE);
    memcpy(GetTextureCoordsPointer() + vi,
	   texture_coord, sizeof(float)*TM_TEXTURE_STRIDE);
    ret = vi / TM_TEXTURE_STRIDE;
  }
  return ret;
}


unsigned int
TriangleMesh::AppendVertexData( const Eigen::Vector3f * pVertex, const Eigen::Vector3f * pNormal,
							    bool bFlipNormal, const Eigen::Vector2f * pTextureCoord, const Eigen::Vector3f * pColor )
{
#if TM_VERTEX_STRIDE != 4
#error fix for different stride
#endif
  unsigned int vi = 0;
  unsigned int ret = 0;
  if(pVertex != NULL){
    vi = (unsigned int)tm_vertices.size();
    tm_vertices.resize(vi+TM_VERTEX_STRIDE);
	tm_vertices[vi] = pVertex->x();
	tm_vertices[vi+1] = pVertex->y();
	tm_vertices[vi+2] = pVertex->z();
	tm_vertices[vi+3] = 1.0f;
    ret = vi / TM_VERTEX_STRIDE;
  }
  if(pNormal != NULL){
    vi = (unsigned int)tm_normals.size();
    tm_normals.resize(vi+TM_VERTEX_STRIDE);
	float fFlip = (bFlipNormal) ? -1.0f : 1.0f;
	tm_normals[vi] = pNormal->x() * fFlip;
	tm_normals[vi+1] = pNormal->y() * fFlip;
	tm_normals[vi+2] = pNormal->z() * fFlip;
	tm_normals[vi+3] = 1.0f;
    ret = vi / TM_VERTEX_STRIDE;
  }
  if(pColor != NULL){
    vi = (unsigned int)tm_colors.size();
    tm_colors.resize(vi+TM_VERTEX_STRIDE);
	tm_colors[vi] = pColor->x();
	tm_colors[vi+1] = pColor->y();
	tm_colors[vi+2] = pColor->z();
	tm_colors[vi+3] = 1.0f;
    ret = vi / TM_VERTEX_STRIDE;
  }

  if(pTextureCoord != NULL){
    unsigned int ti = (unsigned int)tm_texture_coords.size();
    tm_texture_coords.resize(ti+TM_TEXTURE_STRIDE);
	tm_texture_coords[ti] = pTextureCoord->x();
	tm_texture_coords[ti+1] = pTextureCoord->y();
  }

  return ret;
}


unsigned int
TriangleMesh::AppendTriangleData(unsigned int * triangle)
{
  unsigned int ti = 0;
  if(triangle != NULL){
    ti = (unsigned int)tm_triangles.size();
    tm_triangles.resize(ti+TM_TRIANGLE_STRIDE);
    memcpy(GetTrianglePointer() + ti,
	   triangle, sizeof(unsigned int)*TM_TRIANGLE_STRIDE);
  }
  return ti/TM_TRIANGLE_STRIDE;
}



//void TriangleMesh::GetVertex( unsigned int nVertex, Eigen::Vector3f & v ) const
//{
//	v.x() = tm_vertices[ nVertex * TM_VERTEX_STRIDE ];
//	v.y() = tm_vertices[ nVertex * TM_VERTEX_STRIDE + 1];
//	v.z() = tm_vertices[ nVertex * TM_VERTEX_STRIDE + 2];
//}

void TriangleMesh::GetNormal( unsigned int nNormal, Eigen::Vector3f & n ) const
{
	n.x() = tm_normals[ nNormal * TM_VERTEX_STRIDE ];
	n.y() = tm_normals[ nNormal * TM_VERTEX_STRIDE + 1];
	n.z() = tm_normals[ nNormal * TM_VERTEX_STRIDE + 2];
}

void TriangleMesh::GetTextureCoords( unsigned int nVertex, Eigen::Vector2f & vt ) const
{
	vt.x() = tm_texture_coords[ nVertex * TM_TEXTURE_STRIDE ];
	vt.y() = tm_texture_coords[ nVertex * TM_TEXTURE_STRIDE + 1];
}


void TriangleMesh::GetTriangle( unsigned int nTriangle, unsigned int triangle[3] ) const
{
	memcpy(triangle, & tm_triangles[ nTriangle * TM_TRIANGLE_STRIDE ], sizeof(unsigned int)*3 );
}

//void TriangleMesh::GetTriangle( unsigned int nTriangle, Eigen::Vector3f * vVertices ) const
//{
//	GetVertex( tm_triangles[nTriangle * TM_TRIANGLE_STRIDE], vVertices[0] );
//	GetVertex( tm_triangles[nTriangle * TM_TRIANGLE_STRIDE + 1], vVertices[1] );
//	GetVertex( tm_triangles[nTriangle * TM_TRIANGLE_STRIDE + 2], vVertices[2] );
//}

void TriangleMesh::GetTriangleNormals( unsigned int nTriangle, Eigen::Vector3f * vNormals )
{
	GetNormal( tm_triangles[nTriangle * TM_TRIANGLE_STRIDE], vNormals[0] );
	GetNormal( tm_triangles[nTriangle * TM_TRIANGLE_STRIDE + 1], vNormals[1] );
	GetNormal( tm_triangles[nTriangle * TM_TRIANGLE_STRIDE + 2], vNormals[2] );
}

void TriangleMesh::GetTextureCoords( unsigned int nTriangle, Eigen::Vector2f * vCoords ) const
{
	GetTextureCoords( tm_triangles[nTriangle * TM_TRIANGLE_STRIDE], vCoords[0] );
	GetTextureCoords( tm_triangles[nTriangle * TM_TRIANGLE_STRIDE + 1], vCoords[1] );
	GetTextureCoords( tm_triangles[nTriangle * TM_TRIANGLE_STRIDE + 2], vCoords[2] );
}

void TriangleMesh::GetTriTexCoords( unsigned int nTriangle, Eigen::Vector2f & uv1, Eigen::Vector2f & uv2, Eigen::Vector2f & uv3 ) const
{
	unsigned int nOffset = TM_TRITEXCOORD_INDEX(nTriangle, 0);
	uv1 = Eigen::Vector2f( tm_triTexCoords[ nOffset   ], tm_triTexCoords[ nOffset+1 ] );
	uv2 = Eigen::Vector2f( tm_triTexCoords[ nOffset+2 ], tm_triTexCoords[ nOffset+3 ] );
	uv3 = Eigen::Vector2f( tm_triTexCoords[ nOffset+4 ], tm_triTexCoords[ nOffset+5 ] );
}

void TriangleMesh::SetTriangle( unsigned int nTriangle, unsigned int triangle[3] )
{
	memcpy(& tm_triangles[ nTriangle * TM_TRIANGLE_STRIDE ], triangle, sizeof(unsigned int)*3 );
}

void TriangleMesh::SetVertex( unsigned int nVertex, const Eigen::Vector3f & vVertex )
{
	tm_vertices[ nVertex * TM_VERTEX_STRIDE ] = vVertex.x();
	tm_vertices[ nVertex * TM_VERTEX_STRIDE + 1] = vVertex.y();
	tm_vertices[ nVertex * TM_VERTEX_STRIDE + 2] = vVertex.z();
}

void TriangleMesh::SetNormal( unsigned int nVertex, const Eigen::Vector3f & vNormal )
{
	tm_normals[ nVertex * TM_VERTEX_STRIDE ] = vNormal.x();
	tm_normals[ nVertex * TM_VERTEX_STRIDE + 1] = vNormal.y();
	tm_normals[ nVertex * TM_VERTEX_STRIDE + 2] = vNormal.z();
}



void
TriangleMesh::Reserve(unsigned int mask, unsigned int new_size)
{
  if(mask & TM_VERTEX_BIT)
    tm_vertices.reserve(new_size*TM_VERTEX_STRIDE);
  if(mask & TM_NORMAL_BIT)
    tm_normals.reserve(new_size*TM_VERTEX_STRIDE);
  if(mask & TM_COLOR_BIT)
    tm_colors.reserve(new_size*TM_VERTEX_STRIDE);
  if(mask & TM_TEXTURE_BIT)
    tm_texture_coords.reserve(new_size*TM_TEXTURE_STRIDE);
  if(mask & TM_TRIANGLE_BIT)
    tm_triangles.reserve(new_size*TM_TRIANGLE_STRIDE);
  if(mask & TM_TRIANGLE_TEXCOORD_BIT)
	  tm_triTexCoords.reserve(new_size*TM_TRITEXCOORD_STRIDE);
}


void
TriangleMesh::Resize(unsigned int mask, unsigned int new_size)
{
  if(mask & TM_VERTEX_BIT)
    tm_vertices.resize(new_size*TM_VERTEX_STRIDE);
  if(mask & TM_NORMAL_BIT)
    tm_normals.resize(new_size*TM_VERTEX_STRIDE);
  if(mask & TM_COLOR_BIT)
    tm_colors.resize(new_size*TM_VERTEX_STRIDE);
  if(mask & TM_TEXTURE_BIT)
    tm_texture_coords.resize(new_size*TM_TEXTURE_STRIDE);
  if(mask & TM_TRIANGLE_BIT)
    tm_triangles.resize(new_size*TM_TRIANGLE_STRIDE);
  if(mask & TM_TRIANGLE_TEXCOORD_BIT)
	  tm_triTexCoords.resize(new_size*TM_TRITEXCOORD_STRIDE);
}


void TriangleMesh::Clear( unsigned int nBitMask )
{
  if(nBitMask & TM_VERTEX_BIT)
    tm_vertices.resize(0);
  if(nBitMask & TM_NORMAL_BIT)
    tm_normals.resize(0);
  if(nBitMask & TM_COLOR_BIT)
    tm_colors.resize(0);
  if(nBitMask & TM_TEXTURE_BIT)
    tm_texture_coords.resize(0);
  if(nBitMask & TM_TRIANGLE_BIT)
    tm_triangles.resize(0);
  if(nBitMask & TM_TRIANGLE_TEXCOORD_BIT)
	  tm_triTexCoords.resize(0);
}



void
TriangleMesh::GetBoundingBox( float box[6] )
{
	box[0] = box[2] = box[4] = FLT_MAX;
	box[1] = box[3] = box[5] = -FLT_MAX;

	if(tm_vertices.size() < TM_VERTEX_STRIDE-1) {
		return;
	}

	unsigned int nCount = (unsigned int)tm_vertices.size()/TM_VERTEX_STRIDE - 1;
	for ( unsigned int i = 0; i < nCount; ++i) {

		if ( tm_vertices[ TM_VERTEX_INDEX(i,0) ] < box[0] )
			box[0] = tm_vertices[ TM_VERTEX_INDEX(i,0) ];
		if ( tm_vertices[ TM_VERTEX_INDEX(i,0) ] > box[1] )
			box[1] = tm_vertices[ TM_VERTEX_INDEX(i,0) ];

		if ( tm_vertices[ TM_VERTEX_INDEX(i,1) ] < box[2] )
			box[2] = tm_vertices[ TM_VERTEX_INDEX(i,1) ];
		if ( tm_vertices[ TM_VERTEX_INDEX(i,1) ] > box[3] )
			box[3] = tm_vertices[ TM_VERTEX_INDEX(i,1) ];

		if ( tm_vertices[ TM_VERTEX_INDEX(i,2) ] < box[4] )
			box[4] = tm_vertices[ TM_VERTEX_INDEX(i,2) ];
		if ( tm_vertices[ TM_VERTEX_INDEX(i,2) ] > box[5] )
			box[5] = tm_vertices[ TM_VERTEX_INDEX(i,2) ];
	}
}

float TriangleMesh::GetMaxEdgeLength() const
{
	float fMaxEdgeLength = 0.0f;

	unsigned int nTris = GetNumTriangles();
	for (unsigned int i = 0; i < nTris; ++i) {
		Eigen::Vector3f vVertices[3];
		GetTriangle(i, vVertices);
		for ( int i = 0; i < 3; ++i ) {
			float fLen = (vVertices[i] - vVertices[(i+1)%3]).squaredNorm();
			if ( fLen > fMaxEdgeLength ) {
				fMaxEdgeLength = fLen;
			}
		}
	}
	return (float)sqrt(fMaxEdgeLength);
}



void
TriangleMesh::tm_initialize(unsigned int vertex_sizehint, unsigned int triangle_sizehint)
{
  if(vertex_sizehint != 0){
    tm_vertices.reserve(vertex_sizehint);
    tm_normals.reserve(vertex_sizehint);
    tm_colors.reserve(vertex_sizehint);
    tm_texture_coords.reserve(vertex_sizehint);
  }
  if(triangle_sizehint != 0)
    tm_triangles.reserve(triangle_sizehint);

  tm_fileerror = string("no error");
}



bool TriangleMesh::tm_readobj()
{
	ifstream in(tm_filename.c_str());
	if(!in){
		tm_fileerror = string("Cannot open file ") + tm_filename;
		cerr << tm_fileerror << endl;
		return false;
	}

	string command;
	char c1,c2;
	unsigned int tv1, tv2, tv3, tn1, tn2, tn3, tt1, tt2, tt3;
	char linebuf[1024];

	rmsmesh::Point fvec;
	fvec.init(0.0f, 0.0f, 0.0f);

	bool bHasTextures = false;

	// need to save normals separately and then match to vertices (maya
	//  "optimizes" the mesh...argh!)
	std::vector<Eigen::Vector3f> vNormals;

	bool bHasNormals = false;

	unsigned int ivec[3];
	while(in){
		ostrstream s;
		in >> command;
		if(!in)
			continue;
		switch(command.c_str()[0]){

		case 'v':
			in >> fvec[0] >> fvec[1];
			if(!in)
				continue;
			switch(command.c_str()[1]){
			case '\0':  // vertex
				in >> fvec[2];
				AppendVertexData(fvec);
				break;
			case 'n': // vertex normal
				bHasNormals = true;
				in >> fvec[2];
				fvec.normalize();
				vNormals.push_back( Eigen::Vector3f( fvec[0], fvec[1], fvec[2] ) );
				break;
			case 't':
//				AppendVertexData(NULL, NULL, NULL, fVec);
				bHasTextures = true;
				break;
			default:
				string err("Got unknown OBJ command ");
				err += command;
				cerr << err << endl;
			}
		break;

		case 'f':
			if ( bHasTextures ) {
				in >> tv1 >> c1 >> tt1 >> c2 >> tn1;
				in >> tv2 >> c1 >> tt2 >> c2 >> tn2;
				in >> tv3 >> c1 >> tt3 >> c2 >> tn3;

			}
			else if ( bHasNormals ) {
				in >> tv1 >> c1 >> c2 >> tn1;
				in >> tv2 >> c1 >> c2 >> tn2;
				in >> tv3 >> c1 >> c2 >> tn3;
			}
			else {
				in >> tv1;
				in >> tv2;
				in >> tv3;
			}
			ivec[0] = tv1-1; ivec[1] = tv2-1; ivec[2] = tv3-1;
			AppendTriangleData(ivec);

			// set proper normal
			//AddVertexData( ivec[0], NULL, vNormals[ tn1-1 ] );
			//AddVertexData( ivec[1], NULL, vNormals[ tn2-1 ] );
			//AddVertexData( ivec[2], NULL, vNormals[ tn3-1 ] );

			// [RMS] ignorning normals for IgarashiDeform2D app
			//AddVertexData( ivec[0], NULL, NULL );
			//AddVertexData( ivec[1], NULL, NULL );
			//AddVertexData( ivec[2], NULL, NULL );


			break;

		default:
			in.getline(linebuf, 1023, '\n');
		}
	}

	return true;
}



bool TriangleMesh::tm_writeobj()
{
	ofstream out(tm_filename.c_str());
	if (!out)
		return false;

	bool bHaveVertexTexCoords = HasVertexTextureCoords();
	bool bHaveTriangleTexCoords = HasTriangleTextureCoords();

	// triangle texcoords override vtx texcoords
	if (bHaveTriangleTexCoords)
		bHaveVertexTexCoords = false;

	unsigned int nVerts = GetNumVertices();
	Eigen::Vector3f vert, norm;
	Eigen::Vector2f tex;
	for (unsigned int i = 0; i < nVerts; ++i) {
		GetVertex(i, vert);
		GetNormal(i, norm);
		out << "v " << vert.x() << " " << vert.y() << " " << vert.z() << endl;
		out << "vn " << norm.x() << " " << norm.y() << " " << norm.z() << endl;
		if ( bHaveVertexTexCoords ) {
			GetTextureCoords(i, tex);
			out << "vt " << tex.x() << " " << tex.y() << endl;
		}
	}

	unsigned int nTris = GetNumTriangles();

	if (bHaveTriangleTexCoords)  {
		Eigen::Vector2f vUV[3];
		for ( unsigned int i = 0; i < nTris; ++i ) {
			GetTriTexCoords( i, vUV[0], vUV[1], vUV[2] );

			out << "vt " << vUV[0].x() << " " << vUV[0].y() << std::endl;
			out << "vt " << vUV[1].x() << " " << vUV[1].y() << std::endl;
			out << "vt " << vUV[2].x() << " " << vUV[2].y() << std::endl;
		}
	}

	unsigned int tri[3];
	for (unsigned int i = 0; i < nTris; ++i) {
		GetTriangle(i, tri);
		if ( bHaveVertexTexCoords ) {
			out << "f " << (tri[0]+1) << "/" << (tri[0]+1) << "/" << (tri[0]+1)
				<< " " << (tri[1]+1) << "/" << (tri[1]+1) << "/" << (tri[1]+1)
				<< " " << (tri[2]+1) << "/" << (tri[2]+1) << "/" << (tri[2]+1) << endl;
		} else if ( bHaveTriangleTexCoords) {
			out << "f " << (tri[0]+1) << "/" << ((i*3)+1) << "/" << (tri[0]+1)
				<< " " << (tri[1]+1) << "/" <<  ((i*3)+2) << "/" << (tri[1]+1)
				<< " " << (tri[2]+1) << "/" <<  ((i*3)+3) << "/" << (tri[2]+1) << endl;

		} else {
			out << "f " << (tri[0]+1) << "//" << (tri[0]+1)
				<< " " << (tri[1]+1) << "//" << (tri[1]+1)
				<< " " << (tri[2]+1) << "//" << (tri[2]+1) << endl;
		}
	}

	out.close();

	tm_fileerror = string("no error");
	return true;
}

bool TriangleMesh::tm_writeMeshLite()
{
	ofstream out(tm_filename.c_str());
	if (!out)
		return false;

	bool bHaveTexCoords = GetTextureCoords().size() > 0;

	// write header
	out << "PMeshLite" << std::endl;

	unsigned int nVerts = GetNumVertices();

	// normals??
	out << nVerts << std::endl;

	Eigen::Vector3f norm;
	for (unsigned int i = 0; i < nVerts; ++i) {
		GetNormal(i, norm);
		out << norm.x() << " " << norm.y() << " " << norm.z() << endl;
	}


	// start mesh
	out << "StartMesh" << std::endl;

	Eigen::Vector3f vert;
	Eigen::Vector2f tex;
	for (unsigned int i = 0; i < nVerts; ++i) {
		GetVertex(i, vert);
		out << "Vertex " << (i+1) << " " << vert.x() << " " << vert.y() << " " << vert.z() << std::endl;
		//if ( bHaveTexCoords ) {
		//	GetTextureCoords(i, tex);
		//	out << "vt " << tex.x() << " " << tex.y() << endl;
		//}
	}

	unsigned int nTris = GetNumTriangles();
	unsigned int tri[3];
	for (unsigned int i = 0; i < nTris; ++i) {
		GetTriangle(i, tri);
		out << "Face " << (i+1) << " " << (tri[0]+1) << " " << (tri[1]+1) << " " << (tri[2]+1) << std::endl;
		//if ( bHaveTexCoords ) {
		//	out << "f " << (tri[0]+1) << "/" << (tri[0]+1) << "/" << (tri[0]+1)
		//		<< " " << (tri[1]+1) << "/" << (tri[1]+1) << "/" << (tri[1]+1)
		//		<< " " << (tri[2]+1) << "/" << (tri[2]+1) << "/" << (tri[2]+1) << endl;
		//} else {
		//	out << "f " << (tri[0]+1) << "//" << (tri[0]+1)
		//		<< " " << (tri[1]+1) << "//" << (tri[1]+1)
		//		<< " " << (tri[2]+1) << "//" << (tri[2]+1) << endl;
		//}
	}

	out << "EndMesh" << std::endl;

	out.close();

	tm_fileerror = string("no error");
	return true;
}


void TriangleMesh::EraseTriangles( const std::vector<bool> & vErase)
{
	unsigned int nTriCount = GetNumTriangles();
	if ( vErase.size() != nTriCount )
		return;

	unsigned int ti = 0;
	unsigned int li = 0;
	unsigned int nKeep = 0;
	for ( unsigned int ti = 0; ti < nTriCount; ++ti ) {
		bool bKeep = !vErase[ti];

		if ( bKeep ) {
			tm_triangles[ li++ ] = tm_triangles[ TM_TRIANGLE_INDEX(ti,0) ];
			tm_triangles[ li++ ] = tm_triangles[ TM_TRIANGLE_INDEX(ti,1) ];
			tm_triangles[ li++ ] = tm_triangles[ TM_TRIANGLE_INDEX(ti,2) ];
			++nKeep;
		}
	}

	// resize the triangle vector
	tm_triangles.resize( nKeep * TM_TRIANGLE_STRIDE );
}
