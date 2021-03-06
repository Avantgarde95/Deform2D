#define FREEGLUT_STATIC
#include <GL/glut.h>

#include <Deform2D.h>

#include "TriangleMesh.h"

#include <set>

rmsmesh::TriangleMesh m_mesh;
float m_bounds[6];
Eigen::Vector2f m_vTranslate;
float m_fScale;
rmsmesh::TriangleMesh m_deformedMesh;

void* m_deformer;
bool m_bConstraintsValid;

std::set<unsigned int> m_vSelected;
unsigned int m_nSelected;

int m_nViewport[4];


// predecls
void InitializeDeformedMesh();
void UpdateDeformedMesh();
void InvalidateConstraints();
void ValidateConstraints();
unsigned int FindHitVertex( float nX, float nY );


void MakeSquareMesh()
{
	m_mesh.Clear();

	const unsigned int nRowLen = 5;

	float fYStep = 2.0f / (float)(nRowLen-1);
	float fXStep = 2.0f / (float)(nRowLen-1);

	for ( unsigned int yi = 0; yi < nRowLen; ++yi ) {
		float fY = -1.0f + (float)yi * fYStep;
		for ( unsigned int xi = 0; xi < nRowLen; ++xi ) {
			float fX = -1.0f + (float)xi * fXStep;

			Eigen::Vector3f vVert { {fX,fY,0} };
			m_mesh.AppendVertex( vVert );
		}
	}

	for ( unsigned int yi = 0; yi < nRowLen-1; ++yi ) {
		unsigned int nRow1 = yi * nRowLen;
		unsigned int nRow2 = (yi+1) * nRowLen;

		for ( unsigned int xi = 0; xi < nRowLen-1; ++xi ) {

			unsigned int nTri1[3] = { nRow1 + xi, nRow2 + xi + 1, nRow1 + xi + 1 };
			unsigned int nTri2[3] = { nRow1 + xi, nRow2 + xi, nRow2 + xi + 1 };

			m_mesh.AppendTriangleData( nTri1 );
			m_mesh.AppendTriangleData( nTri2 );
		}
	}

	InitializeDeformedMesh();
	glutPostRedisplay();
}



void UpdateScale()
{
	glGetIntegerv(GL_VIEWPORT, m_nViewport);
	float fViewCenterX = (float)m_nViewport[2] / 2;
	float fViewCenterY = (float)m_nViewport[3] / 2;

	m_mesh.GetBoundingBox( m_bounds );
	m_vTranslate.x() = fViewCenterX  -  0.5f * ( m_bounds[0] + m_bounds[1] );
	m_vTranslate.y() = fViewCenterY  -  0.5f * ( m_bounds[2] + m_bounds[3] );

	float fWidth = m_bounds[1] - m_bounds[0];
	float fHeight = m_bounds[3] - m_bounds[2];
	float fSizeObject = std::max(fWidth,fHeight);
	float fSizeView = std::min( m_nViewport[2], m_nViewport[3] );

	m_fScale = 0.5f * (fSizeView / fSizeObject);
}
Eigen::Vector2f ViewToWorld( const Eigen::Vector2f & vPoint )
{
	return (vPoint - m_vTranslate) / m_fScale;
}
Eigen::Vector2f WorldToView( const Eigen::Vector2f & vPoint )
{
	return ( vPoint * m_fScale ) + m_vTranslate;
}




void InitializeDeformedMesh()
{
	std::vector<Deform2D_Vector3> verticesToSubmit;
	std::vector<int> facesToSubmit;

	m_deformedMesh.Clear();

	unsigned int nVerts = m_mesh.GetNumVertices();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Eigen::Vector3f vVertex;
		m_mesh.GetVertex(i, vVertex);
		m_deformedMesh.AppendVertex(vVertex);

		verticesToSubmit.push_back({ vVertex.x(), vVertex.y(), 0.0f });
	}

	unsigned int nTris = m_mesh.GetNumTriangles();
	for ( unsigned int i = 0; i < nTris; ++i ) {
		unsigned int nTriangle[3];
		m_mesh.GetTriangle(i,nTriangle);
		m_deformedMesh.AppendTriangleData(nTriangle);

		facesToSubmit.emplace_back(nTriangle[0]);
		facesToSubmit.emplace_back(nTriangle[1]);
		facesToSubmit.emplace_back(nTriangle[2]);
	}

	//m_deformer.InitializeFromMesh( &m_mesh );
	Deform2D_SetMesh(m_deformer, verticesToSubmit.data(), verticesToSubmit.size(), facesToSubmit.data(), facesToSubmit.size() / 3);
	InvalidateConstraints();
}



void UpdateDeformedMesh()
{
	ValidateConstraints();
	//m_deformer.UpdateDeformedMesh( &m_deformedMesh, true );

	std::vector<Deform2D_Vector3> deformedVertices(m_deformedMesh.GetVertexCount());
	std::vector<int> deformedFaces(m_deformedMesh.GetTriangleCount() * 3);

	Deform2D_GetDeformedMesh(m_deformer, deformedVertices.data(), deformedVertices.size(), true);

	for (int i = 0; i < m_deformedMesh.GetVertexCount(); i++) {
		m_deformedMesh.SetVertex(i, {
			deformedVertices[i].x,
			deformedVertices[i].y,
			deformedVertices[i].z
		});
	}
}


// deformer stuff
void InvalidateConstraints()
{
	m_bConstraintsValid = false;
}

void ValidateConstraints()
{
	if ( m_bConstraintsValid )
		return;

	size_t nConstraints = m_vSelected.size();
	std::set<unsigned int>::iterator cur(m_vSelected.begin()), end(m_vSelected.end());
	while ( cur != end ) {
		unsigned int nVertex = *cur++;
		Eigen::Vector3f vVertex;
		m_deformedMesh.GetVertex( nVertex, vVertex);
		Deform2D_Vector3 vertexToSubmit = { vVertex.x(), vVertex.y(), 0.0f };
		Deform2D_SetDeformedHandle(m_deformer, nVertex, &vertexToSubmit);
	}

	Deform2D_ForceValidation(m_deformer);

	m_bConstraintsValid = true;
}


// selection stuff

unsigned int FindHitVertex( float nX, float nY )
{
	unsigned int nVerts = m_deformedMesh.GetNumVertices();
	for ( unsigned int i = 0; i < nVerts; ++i ) {

		Eigen::Vector3f vVertex;
		m_deformedMesh.GetVertex(i, vVertex);
		Eigen::Vector2f vView = WorldToView( Eigen::Vector2f(vVertex.x(), vVertex.y()) );
		float fX = vView.x();
		float fY = vView.y();

		double fDist = sqrt(
			(double)((nX - fX)*(nX - fX) + (nY-fY)*(nY-fY) ));
		if ( fDist < 5 )
			return i;
	}

	return std::numeric_limits<unsigned int>::max();
}






void OnMouseClick(int button, int state, int x, int y)
{
	if ( button == GLUT_LEFT_BUTTON ) {

		if ( state == GLUT_DOWN )
			m_nSelected = FindHitVertex( (float)x, (float)(m_nViewport[3] - 1 - y) );
		else
			m_nSelected = std::numeric_limits<unsigned int>::max();
		glutPostRedisplay();

	} else if ( button == GLUT_RIGHT_BUTTON && state == GLUT_UP ) {

		unsigned int nHit = FindHitVertex( (float)x, (float)(m_nViewport[3] - 1 - y) );
		if ( nHit != std::numeric_limits<unsigned int>::max() ) {

			if ( m_vSelected.find(nHit) == m_vSelected.end() )
				m_vSelected.insert(nHit);
			else {
				m_vSelected.erase(nHit);
				Deform2D_RemoveHandle(m_deformer, nHit);

				// restore position
				Eigen::Vector3f vVertex;
				m_mesh.GetVertex(nHit,vVertex);
				m_deformedMesh.SetVertex(nHit, vVertex);
			}

			InvalidateConstraints();
			glutPostRedisplay();
		}

	}

}



void OnMouseMove(int x, int y)
{
	if ( m_nSelected != std::numeric_limits<unsigned int>::max() ) {
		Eigen::Vector2f vNewPosView( (float)x, (float)(m_nViewport[3] - 1 - y) );
		Eigen::Vector2f vNewPosWorld = ViewToWorld( vNewPosView );
		Eigen::Vector3f vNewPos( vNewPosWorld.x(), vNewPosWorld.y(), 0.0f );
		m_deformedMesh.SetVertex( m_nSelected, vNewPos );
		InvalidateConstraints();
		glutPostRedisplay();
	}
}


void OnKeyboard(unsigned char key, int x, int y)
{
	if ( key == 'f' ) {
		std::cout << "Mesh path (.obj): ";
		std::string meshPath;
		std::cin >> meshPath;
		m_mesh.Clear();
		m_mesh.read( meshPath.c_str() );
		m_vSelected.clear();
		InitializeDeformedMesh();

		glutPostRedisplay();
	}
}





void OnRender()
{
	UpdateDeformedMesh();

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear( GL_COLOR_BUFFER_BIT );
	glDisable(GL_DEPTH_TEST);

	UpdateScale();
	glLoadIdentity();
	glTranslatef(m_vTranslate.x(), m_vTranslate.y(), 0.0f);
	glScalef(m_fScale, m_fScale, 1.0f);

	glLineWidth(2.0f);
	glColor3f(0.0f, 0.0f, 0.0f);

	unsigned int nTris = m_deformedMesh.GetNumTriangles();
	for ( unsigned int i = 0; i < nTris; ++i ) {
		Eigen::Vector3f vVerts[3];
		m_deformedMesh.GetTriangle(i, vVerts);

		glBegin(GL_LINE_LOOP);
		glVertex3fv( vVerts[0].data() );
		glVertex3fv( vVerts[1].data() );
		glVertex3fv( vVerts[2].data() );
		glEnd();
	}

	glLoadIdentity();
	glColor3f(1.0f, 0.0f, 0.0f);

	std::set<unsigned int>::iterator cur(m_vSelected.begin()), end(m_vSelected.end());
	while ( cur != end ) {
		unsigned int nSelected = *cur++;

		Eigen::Vector3f vSelected;
		m_deformedMesh.GetVertex( nSelected, vSelected );
		Eigen::Vector2f vView = WorldToView( Eigen::Vector2f(vSelected.x(),vSelected.y()) );

		glBegin(GL_QUADS);
		glVertex2f( vView.x() - 5, vView.y() - 5 );
		glVertex2f( vView.x() + 5, vView.y() - 5 );
		glVertex2f( vView.x() + 5, vView.y() + 5 );
		glVertex2f( vView.x() - 5, vView.y() + 5 );
		glEnd();
	}

	glutSwapBuffers();
}




void OnReshape( int width, int height )
{
	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, width, 0, height);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}



int main(int argc, char ** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowPosition(120,260);
	glutInitWindowSize(600,600);
	glutCreateWindow("Deform2D");

	glutDisplayFunc( OnRender );
	glutReshapeFunc( OnReshape );
	glutMouseFunc( OnMouseClick );
	glutMotionFunc( OnMouseMove );
	glutKeyboardFunc( OnKeyboard );

	m_deformer = Deform2D_CreateDeformer();
	MakeSquareMesh();

	glutMainLoop();

	atexit([]() {
		Deform2D_DestroyDeformer(m_deformer);
	});

	return 0;
}

