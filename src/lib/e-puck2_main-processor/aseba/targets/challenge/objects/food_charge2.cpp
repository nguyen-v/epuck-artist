/*
	Challenge - Virtual Robot Challenge System
	Copyright (C) 1999--2008:
		Stephane Magnenat <stephane at magnenat dot net>
		(http://stephane.magnenat.net)
	3D models
	Copyright (C) 2008:
		Basilio Noris
	Aseba - an event-based framework for distributed robot control
	Copyright (C) 2007--2016:
		Stephane Magnenat <stephane at magnenat dot net>
		(http://stephane.magnenat.net)
		and other contributors, see authors.txt for details
	
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published
	by the Free Software Foundation, version 3 of the License.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.
	
	You should have received a copy of the GNU Lesser General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <QtOpenGL>
#define BYTE unsigned char

namespace Enki
{
	// 48 Verticies
	// 74 Texture Coordinates
	// 49 Normals
	// 70 Triangles
	
	static BYTE face_indicies[70][9] = {
	// Object #-1
		{28,18,30 ,0,0,0 ,0,1,2 }, {47,18,28 ,0,0,0 ,3,1,0 }, {47,20,18 ,0,0,0 ,3,4,1 },
		{45,20,47 ,0,0,0 ,5,4,3 }, {45,11,20 ,0,0,0 ,5,6,4 }, {44,11,45 ,0,0,0 ,7,6,5 },
		{44,13,11 ,0,0,0 ,7,8,6 }, {41,13,44 ,0,0,0 ,9,8,7 }, {41,7,13 ,0,0,0 ,9,10,8 },
		{39,7,41 ,0,0,0 ,11,10,9 }, {39,9,7 ,0,0,0 ,11,12,10 }, {37,9,39 ,0,0,0 ,13,12,11 },
		{37,2,9 ,0,0,0 ,13,14,12 }, {35,2,37 ,0,0,0 ,15,14,13 }, {2,35,3 ,0,0,0 ,14,15,16 },
		{3,35,0 ,0,0,0 ,16,15,17 }, {35,33,0 ,0,0,0 ,15,18,17 }, {0,33,14 ,0,0,0 ,17,18,19 },
		{33,32,14 ,0,0,0 ,18,20,19 }, {14,32,16 ,0,0,0 ,19,20,21 }, {24,16,32 ,0,0,0 ,22,21,20 },
		{22,16,24 ,0,0,0 ,23,21,22 }, {4,3,5 ,1,2,3 ,24,25,26 }, {4,2,3 ,1,4,2 ,24,27,25 },
		{10,2,4 ,5,4,1 ,28,27,24 }, {10,9,2 ,5,6,4 ,28,29,27 }, {8,9,10 ,7,6,5 ,30,29,28 },
		{8,7,9 ,7,8,6 ,30,31,29 }, {6,7,8 ,9,8,7 ,32,31,30 }, {6,13,7 ,9,10,8 ,32,33,31 },
		{12,13,6 ,11,10,9 ,34,33,32 }, {12,11,13 ,11,12,10 ,34,35,33 },
		{21,11,12 ,13,12,11 ,36,35,34 }, {21,20,11 ,13,14,12 ,36,37,35 },
		{19,20,21 ,15,14,13 ,38,37,36 }, {19,18,20 ,15,16,14 ,38,39,37 },
		{31,18,19 ,17,16,15 ,40,39,38 }, {31,30,18 ,17,18,16 ,40,41,39 },
		{29,30,31 ,19,18,17 ,42,43,44 }, {29,28,30 ,19,20,18 ,42,45,43 },
		{27,28,29 ,21,20,19 ,46,45,42 }, {27,47,28 ,21,22,20 ,46,47,45 },
		{46,47,27 ,23,22,21 ,48,47,46 }, {46,45,47 ,23,24,22 ,48,49,47 },
		{43,45,46 ,25,24,23 ,50,49,48 }, {43,44,45 ,25,26,24 ,50,51,49 },
		{42,44,43 ,27,26,25 ,52,51,50 }, {42,41,44 ,27,28,26 ,52,53,51 },
		{40,41,42 ,29,28,27 ,54,53,52 }, {40,39,41 ,29,30,28 ,54,55,53 },
		{38,39,40 ,31,30,29 ,56,55,54 }, {38,37,39 ,31,32,30 ,56,57,55 },
		{36,37,38 ,33,32,31 ,58,57,56 }, {36,35,37 ,33,34,32 ,58,59,57 },
		{34,35,36 ,35,34,33 ,60,59,58 }, {34,33,35 ,35,36,34 ,60,61,59 },
		{26,33,34 ,37,36,35 ,62,61,60 }, {26,32,33 ,37,38,36 ,62,63,61 },
		{25,32,26 ,39,38,37 ,64,63,62 }, {25,24,32 ,39,40,38 ,64,65,63 },
		{23,24,25 ,41,40,39 ,66,65,64 }, {23,22,24 ,41,42,40 ,66,67,65 },
		{17,22,23 ,43,42,41 ,68,67,66 }, {17,16,22 ,43,44,42 ,68,69,67 },
		{15,16,17 ,45,44,43 ,70,69,68 }, {15,14,16 ,45,46,44 ,70,71,69 },
		{1,14,15 ,47,46,45 ,72,71,70 }, {1,0,14 ,47,48,46 ,72,73,71 },
		{5,0,1 ,3,48,47 ,26,73,72 }, {5,3,0 ,3,2,48 ,26,25,73 }
	};
	static GLfloat vertices [48][3] = {
	{0.0155634f,0.0155634f,0.09225f},{0.0150897f,0.0150897f,0.0835f},{0.00569661f,0.02126f,0.09225f},
	{0.011005f,0.0190612f,0.09225f},{0.0055232f,0.0206129f,0.0835f},{0.01067f,0.018481f,0.0835f},
	{-0.01067f,0.018481f,0.0835f},{-0.00569661f,0.02126f,0.09225f},{-0.0055232f,0.0206129f,0.0835f},
	{0.0f,0.02201f,0.09225f},{0.0f,0.02134f,0.0835f},{-0.0155634f,0.0155634f,0.09225f},
	{-0.0150897f,0.0150897f,0.0835f},{-0.011005f,0.0190612f,0.09225f},{0.0190612f,0.011005f,0.09225f},
	{0.018481f,0.01067f,0.0835f},{0.02126f,0.00569661f,0.09225f},{0.0206129f,0.0055232f,0.0835f},
	{-0.02126f,0.00569661f,0.09225f},{-0.0206129f,0.0055232f,0.0835f},{-0.0190612f,0.011005f,0.09225f},
	{-0.018481f,0.01067f,0.0835f},{0.02201f,0.0f,0.09225f},{0.02134f,0.0f,0.0835f},
	{0.02126f,-0.00569661f,0.09225f},{0.0206129f,-0.0055232f,0.0835f},{0.018481f,-0.01067f,0.0835f},
	{-0.018481f,-0.01067f,0.0835f},{-0.02126f,-0.00569661f,0.09225f},{-0.0206129f,-0.0055232f,0.0835f},
	{-0.02201f,0.0f,0.09225f},{-0.02134f,0.0f,0.0835f},{0.0190612f,-0.011005f,0.09225f},
	{0.0155634f,-0.0155634f,0.09225f},{0.0150897f,-0.0150897f,0.0835f},{0.011005f,-0.0190612f,0.09225f},
	{0.01067f,-0.018481f,0.0835f},{0.00569661f,-0.02126f,0.09225f},{0.0055232f,-0.0206129f,0.0835f},
	{0.0f,-0.02201f,0.09225f},{0.0f,-0.02134f,0.0835f},{-0.00569661f,-0.02126f,0.09225f},
	{-0.0055232f,-0.0206129f,0.0835f},{-0.01067f,-0.018481f,0.0835f},{-0.011005f,-0.0190612f,0.09225f},
	{-0.0155634f,-0.0155634f,0.09225f},{-0.0150897f,-0.0150897f,0.0835f},{-0.0190612f,-0.011005f,0.09225f}
	};
	static GLfloat normals [49][3] = {
	{0.0f,0.0f,1.0f},{0.300042f,0.950872f,-0.076275f},{0.460206f,0.884529f,-0.0762749f},
	{0.535922f,0.840815f,-0.0762749f},{0.215592f,0.9735f,-0.076275f},{0.0437144f,0.996128f,-0.076275f},
	{-0.0437144f,0.996128f,-0.076275f},{-0.215592f,0.9735f,-0.076275f},{-0.300042f,0.950872f,-0.076275f},
	{-0.460206f,0.884529f,-0.0762749f},{-0.535922f,0.840815f,-0.0762749f},{-0.673458f,0.735279f,-0.0762749f},
	{-0.73528f,0.673458f,-0.0762749f},{-0.840815f,0.535922f,-0.0762749f},{-0.884529f,0.460206f,-0.0762749f},
	{-0.950872f,0.300042f,-0.076275f},{-0.9735f,0.215592f,-0.076275f},{-0.996128f,0.0437144f,-0.076275f},
	{-0.996128f,-0.0437144f,-0.076275f},{-0.9735f,-0.215592f,-0.076275f},{-0.950872f,-0.300042f,-0.076275f},
	{-0.884529f,-0.460206f,-0.0762749f},{-0.840815f,-0.535922f,-0.0762749f},{-0.735279f,-0.673458f,-0.0762749f},
	{-0.673458f,-0.73528f,-0.0762749f},{-0.535922f,-0.840815f,-0.0762749f},{-0.460206f,-0.884529f,-0.0762749f},
	{-0.300042f,-0.950872f,-0.076275f},{-0.215592f,-0.9735f,-0.076275f},{-0.0437144f,-0.996128f,-0.076275f},
	{0.0437144f,-0.996128f,-0.076275f},{0.215592f,-0.9735f,-0.076275f},{0.300042f,-0.950872f,-0.076275f},
	{0.460206f,-0.884529f,-0.0762749f},{0.535922f,-0.840815f,-0.0762749f},{0.673458f,-0.735279f,-0.0762749f},
	{0.73528f,-0.673458f,-0.0762749f},{0.840815f,-0.535922f,-0.0762749f},{0.884529f,-0.460206f,-0.0762749f},
	{0.950872f,-0.300042f,-0.076275f},{0.9735f,-0.215592f,-0.076275f},{0.996128f,-0.0437144f,-0.076275f},
	{0.996128f,0.0437144f,-0.076275f},{0.9735f,0.215592f,-0.076275f},{0.950872f,0.300042f,-0.076275f},
	{0.884529f,0.460206f,-0.0762749f},{0.840815f,0.535922f,-0.0762749f},{0.735279f,0.673458f,-0.0762749f},
	{0.673458f,0.73528f,-0.0762749f}
	};
	static GLfloat textures [74][2] = {
	{0.510277f,0.111667f},{0.510277f,0.130689f},{0.509025f,0.121178f},
	{0.513948f,0.102804f},{0.513948f,0.139551f},{0.519788f,0.0951934f},
	{0.519788f,0.147162f},{0.527399f,0.0893536f},{0.527399f,0.153002f},
	{0.536261f,0.0856825f},{0.536261f,0.156673f},{0.545772f,0.0844303f},
	{0.545772f,0.157925f},{0.555283f,0.0856825f},{0.555283f,0.156673f},
	{0.564146f,0.0893536f},{0.564146f,0.153002f},{0.571757f,0.147162f},
	{0.571757f,0.0951934f},{0.577597f,0.139551f},{0.577597f,0.102804f},
	{0.581268f,0.130689f},{0.581268f,0.111667f},{0.58252f,0.121178f},
	{0.624689f,0.226675f},{0.613751f,0.258856f},{0.613751f,0.226675f},
	{0.624689f,0.258856f},{0.635626f,0.226675f},{0.635626f,0.258856f},
	{0.646564f,0.226675f},{0.646564f,0.258856f},{0.657502f,0.226675f},
	{0.657502f,0.258856f},{0.66844f,0.226675f},{0.66844f,0.258856f},
	{0.679377f,0.226675f},{0.679377f,0.258856f},{0.690315f,0.226675f},
	{0.690315f,0.258856f},{0.701253f,0.226675f},{0.701253f,0.258856f},
	{0.449685f,0.226675f},{0.438747f,0.257813f},{0.438747f,0.226675f},
	{0.449685f,0.258856f},{0.460623f,0.226675f},{0.460623f,0.258856f},
	{0.47156f,0.226675f},{0.47156f,0.258856f},{0.482498f,0.226675f},
	{0.482498f,0.258856f},{0.493436f,0.226675f},{0.493436f,0.258856f},
	{0.504374f,0.226675f},{0.504374f,0.258856f},{0.515311f,0.226675f},
	{0.515311f,0.258856f},{0.526249f,0.226675f},{0.526249f,0.258856f},
	{0.537187f,0.226675f},{0.537187f,0.258856f},{0.548125f,0.226675f},
	{0.548125f,0.258856f},{0.559062f,0.226675f},{0.559062f,0.258856f},
	{0.57f,0.226675f},{0.57f,0.258856f},{0.580938f,0.226675f},
	{0.580938f,0.258856f},{0.591875f,0.226675f},{0.591875f,0.258856f},
	{0.602813f,0.226675f},{0.602813f,0.258856f}
	};
	GLint GenFeederCharge2()
	{
		GLint lid=glGenLists(1);
		glNewList(lid, GL_COMPILE);
		
			glBegin (GL_TRIANGLES);
			for(size_t i=0;i<sizeof(face_indicies)/sizeof(face_indicies[0]);i++)
			{
				for(size_t j=0;j<3;j++)
				{
					int vi=face_indicies[i][j];
					int ni=face_indicies[i][j+3];//Normal index
					int ti=face_indicies[i][j+6];//Texture index
					/*glNormal3f (normals[ni][0],normals[ni][1],normals[ni][2]);
					glTexCoord2f(textures[ti][0],textures[ti][1]);
					glVertex3f (vertices[vi][0],vertices[vi][1],vertices[vi][2]);*/
					
					// rotate 90 deg around z
					glNormal3f (normals[ni][1],-normals[ni][0],normals[ni][2]);
					glTexCoord2f(textures[ti][0],textures[ti][1]);
					glVertex3f (100.f*vertices[vi][1],-100.f*vertices[vi][0],100.f*vertices[vi][2]);
				}
			}
			glEnd ();
		
		glEndList();
		return lid;
	};
}
