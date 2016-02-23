#pragma once

#include "./MeshData.h"
#include "../third_party/ply.h"

#define OBJ_LINE_BUF_SIZE 256

template< class FloatType>
class MeshIO
{
public:

  // To be added, loading mesh from files
  static MeshData<FloatType> loadfromFile(const std::string& filename, bool clockwise = true);

  static void loadfromFile(const std::string& filename, MeshData<FloatType>& meshData,
                           bool clockwise = true);

  static void updateFromFile(const std::string& filename, MeshData<FloatType>& meshData);

  static void writeToFile(const std::string& filename, const MeshData<FloatType>& meshData);

  // create mesh from grid aligned 3d points
  static void createMeshFromDepth(MeshData<FloatType>& meshData,
                                  unsigned char* pColorImageRGB, DepthImageType& uImage,
                                  DepthImageType vImage, DepthImageType& dImage,
                                  InternalIntensityImageType& maskImage, int rows, int cols, double scale = 1.0);

  static void createMeshFromDepth(MeshData<FloatType>& meshData,
                                  InternalColorImageType& colorImage, DepthImageType& uImage,
                                  DepthImageType vImage, DepthImageType& dImage,
                                  InternalIntensityImageType& maskImage, int rows, int cols, double scale = 1.0);

  static void createMeshFromPoints(MeshData<FloatType>& meshData, int rows, int cols,
                                   FloatType* pPointsImage, FloatType* pNormalsImage, FloatType* pColorsImage,
                                   vector<double>& maskImage,  vector<unsigned int>& modelLabels,
                                   vector<FloatType>& modelColors);

  static void updateMesh(MeshData<FloatType>& meshData, int rows, int cols,
                         FloatType* pPointsImage, FloatType* pNormalsImage, FloatType* pColorsImage,
                         vector<double>& maskImage,  vector<unsigned int>& modelLabels,
                         vector<FloatType>& modelColors);


  static void setupMeshFromFile(MeshData<FloatType>& meshData);

private:


  /************************************************************************/
	/* Read Functions													    */
	/************************************************************************/

  static void loadFromPLY(const std::string& filename, MeshData<FloatType>& meshData);

	static void loadFromOFF(const std::string& filename, MeshData<FloatType>& meshData);

	static void loadFromOBJ(const std::string& filename, MeshData<FloatType>& meshData);

  static void updateFromPLY(const std::string& filename, MeshData<FloatType>& meshData);

  static void updateFromOBJ(const std::string& filename, MeshData<FloatType>& meshData);

	/************************************************************************/
	/* Write Functions													    */
	/************************************************************************/

	static void writeToPLY(const std::string& filename, const MeshData<FloatType>& meshData);

	static void writeToOFF(const std::string& filename, const MeshData<FloatType>& meshData) {};

	static void writeToOBJ(const std::string& filename, const MeshData<FloatType>& meshData);

  static void skipLine(char * buf, int size, FILE * fp);

  template<typename VertexType>
  static void ply_read_vertices(PlyFile *_ply, int _vertex_type, int _num_vertices,
	  MeshData<FloatType>& meshData);

};

/// meshIO
template<class FloatType>
MeshData<FloatType> MeshIO<FloatType>::loadfromFile(const std::string& filename, bool clockwise)
{
  MeshData<FloatType> meshData;
  loadfromFile(filename, meshData, clockwise);

  return meshData;
}

template<class FloatType>
void MeshIO<FloatType>::loadfromFile(const std::string& filename,
                                     MeshData<FloatType>& meshData, bool clockwise)
{
  bfs::path filePath(filename.c_str());

  //cout << filePath.extension().c_str() << endl;

  meshData.clockwise = clockwise;

  if(filePath.extension().compare(std::string(".off")) == 0) {
    loadFromOFF(filename,meshData);
  } else if(filePath.extension().compare(std::string(".ply")) == 0){
    loadFromPLY(filename,meshData);
  } else if(filePath.extension().compare(std::string(".obj")) == 0){
    loadFromOBJ(filename,meshData);
  } else {
    cerr << "file format not supported for mesh loading" << endl;
  }

  // we will update the number of vertices, faces and mesh center in the
  // following line
  setupMeshFromFile(meshData);

}
template<class FloatType>
void MeshIO<FloatType>::updateFromFile(const std::string& filename, MeshData<FloatType>& meshData)
{
  bfs::path filePath(filename.c_str());
  //cout << filePath.extension().c_str() << endl;

  if(filePath.extension().compare(std::string(".off")) == 0) {
    cout << "not supported yet" << endl;
  } else if(filePath.extension().compare(std::string(".ply")) == 0){
    // cout << "not supported yet" << endl;
    updateFromPLY(filename,meshData);
  } else if(filePath.extension().compare(std::string(".obj")) == 0){
    updateFromOBJ(filename,meshData);
  } else {
    cerr << "file format not supported for mesh loading" << endl;
  }
}

template<class FloatType>
template<typename VertexType>
void MeshIO<FloatType>::ply_read_vertices(PlyFile *_ply, int _vertex_type, int _num_vertices,
	MeshData<FloatType>& meshData)
{
	/* set up for getting vertex elements */
	for (int i = 0; i < ply::n_vprops[_vertex_type]; i++)
	{
		PlyProperty* prop = ply::get_vertex_property(_vertex_type, i);

		ply_get_property(_ply, ply::elem_names[0], prop);
	}

	meshData.vertices.resize(_num_vertices);
	meshData.colors.resize(_num_vertices);

  if(_vertex_type == PLY_VERTEX_NORMAL_RGB || _vertex_type == PLY_VERTEX_NORMAL_RGBA)
    meshData.normals.resize(_num_vertices);

	/* grab all the vertex elements */
	for (int i = 0; i < _num_vertices; i++) {
		VertexType vertex;

		/* grab an element from the file */
		ply_get_element(_ply, (void *)&vertex);

		vector<FloatType> v = { (FloatType)vertex.x, (FloatType)vertex.y, (FloatType)vertex.z };
		meshData.vertices[i] = std::move(v);

		// For now, normals are not loaded
		//if (_vertex_type & 0x01)
		//{
		//	vector<float> n = { vertex.nx, vertex.ny, vertex.nz };
		// vector<FloatType> n = { 0, 0, 0 };
		// meshData.normals[i] = n;
    if(_vertex_type == PLY_VERTEX_NORMAL_RGB)
      {
        vector<FloatType> n = {(FloatType)((ply::VertexNormalColor*)&vertex)->nx,
                               (FloatType)((ply::VertexNormalColor*)&vertex)->ny,
                               (FloatType)((ply::VertexNormalColor*)&vertex)->nz};
        meshData.normals[i] = std::move(n);
      }

    if(_vertex_type == PLY_VERTEX_NORMAL_RGBA)
      {
        vector<FloatType> n = {(FloatType)((ply::VertexNormalColorAlpha*)&vertex)->nx,
                               (FloatType)((ply::VertexNormalColorAlpha*)&vertex)->ny,
                               (FloatType)((ply::VertexNormalColorAlpha*)&vertex)->nz};
        meshData.normals[i] = std::move(n);
      }

    //cout << meshData.normals[0][0] <<  " " <<  meshData.normals[0][1] << " " <<  meshData.normals[0][2] << endl;

		//}

		vector<FloatType> c = { (FloatType)vertex.r, (FloatType)vertex.g, (FloatType)vertex.b };
		c[0] /= 255.f;
		c[1] /= 255.f;
		c[2] /= 255.f;

		meshData.colors[i] = std::move(c);
	}
}

template<class FloatType>
void MeshIO<FloatType>::loadFromPLY(const std::string& filename,
                                    MeshData<FloatType>& meshData)
{
	PlyFile *ply;
	char **elist;
	int file_type;
	float version;
	int nelems, num_elems, nprops;
	char *elem_name;
	PlyProperty **plist;
	int num_comments;
	char **comments;
	int num_obj_info;
	char **obj_info;

	/* open a PLY file for reading */
	ply = ply_open_for_reading(filename.c_str(), &nelems, &elist, &file_type, &version);

	/* go through each kind of element that we learned is in the file */
	/* and read them */

  unsigned int vertex_type;

	for (int i = 0; i < nelems; i++) {

		/* get the description of the first element */
		elem_name = elist[i];
		plist = ply_get_element_description(ply, elem_name, &num_elems, &nprops);

		/* if we're on vertex elements, read them in */
		if (equal_strings(ply::elem_names[0], elem_name)) {
			int num_vertices = num_elems;

			vertex_type = ply::get_vertex_type(plist, nprops);

			switch (vertex_type)
			{
			case PLY_VERTEX_RGB:
				ply_read_vertices<ply::VertexColor>(ply, vertex_type, num_vertices,
					meshData);
				break;
			case PLY_VERTEX_NORMAL_RGB:
				ply_read_vertices<ply::VertexNormalColor>(ply, vertex_type, num_vertices,
					meshData);
				break;
			case PLY_VERTEX_RGBA:
				ply_read_vertices<ply::VertexColorAlpha>(ply, vertex_type, num_vertices,
					meshData);
				break;
			case PLY_VERTEX_NORMAL_RGBA:
				ply_read_vertices<ply::VertexNormalColorAlpha>(ply, vertex_type, num_vertices,
					meshData);
				break;
			default:
				break;
			}
		}

		/* if we're on face elements, read them in */
		if (equal_strings(ply::elem_names[1], elem_name)) {
			int num_faces = num_elems;

			/* set up for getting face elements */
			for (int i = 0; i < nprops; i++)
			{
				ply_get_property(ply, elem_name, &ply::face_props[i]);
			}

			meshData.facesVerticesInd.resize(num_faces);

			/* grab all the face elements */
			for (int j = 0; j < num_faces; j++) {
				ply::Face face;
				/* grab and element from the file */
				ply_get_element(ply, (void *)&face);

				vector<unsigned int> f;
				for (int k = 0; k < face.nverts; k++)
				{
					f.push_back(face.verts[k]);
				}
				meshData.facesVerticesInd[j] = f;
			}
		}
	}

	/* grab and print out the comments in the file */
	comments = ply_get_comments(ply, &num_comments);

	/* grab and print out the object information */
	obj_info = ply_get_obj_info(ply, &num_obj_info);

	/* close the PLY file */
	ply_close(ply);

  meshData.numVertices = meshData.vertices.size();
  meshData.numFaces = meshData.facesVerticesInd.size();

  if(vertex_type == PLY_VERTEX_RGBA || vertex_type == PLY_VERTEX_RGB)
    meshData.computeNormalsNeil();

}

template<class FloatType>
void MeshIO<FloatType>::loadFromOFF(const std::string& filename,
                                    MeshData<FloatType>& meshData)
{
  std::ifstream file(filename.c_str());
  if(!file.is_open())
    {
      cerr << "could not open file " << filename << endl;
      return;
    }

  meshData.clear();

  // first line should say 'OFF'
	char string1[5];
	file >> string1;

	// read header
	unsigned int numV = 0;
	unsigned int numP = 0;
	unsigned int numE = 0;
	file >> numV >> numP >> numE;

  meshData.vertices.resize(numV);
  meshData.facesVerticesInd.resize(numP);
  if(numE == 0)
    cout << "no color info in this mesh!" << endl;
  meshData.colors.resize(numV);

  if(std::string(string1).compare("OFF") == 0)
    {
      // no color information ?
      for(unsigned int i = 0; i < numV; ++i) {
        vector<FloatType> v,c;
        v.resize(3);
        c.resize(3);
        file >> v[0] >> v[1] >> v[2];
        meshData.vertices[i] = v;
        c[0] = 0.5;
        c[1] = 0.5;
        c[2] = 0.5;
        meshData.colors[i] = c;
      }
    } else
    {
      // colored mesh
      for(unsigned int i = 0; i < numV; ++i) {
        vector<FloatType> v,c;
        v.resize(3);
        c.resize(3);
        file >> v[0] >> v[1] >> v[2];
        file >> c[0] >> c[1] >> c[2];
        FloatType dummy;
        file >> dummy;
        meshData.vertices[i] = v;
        meshData.colors[i] = c;
      }
    }

  // read faces
  for(unsigned int i = 0; i < numP; ++i){
    unsigned int num_vs; // number of vertices in the face, we only support 3
    file >> num_vs;
    if(num_vs != 3)
      cout << "not triangle face!" << endl;
    for(unsigned int j = 0; j < num_vs; ++j){
      unsigned int idx;
      file >> idx;
      meshData.facesVerticesInd[i].push_back(idx);
    }
  }


}

template<class FloatType>
void MeshIO<FloatType>::loadFromOBJ(const std::string& filename,
                                    MeshData<FloatType>& meshData)
{

  FILE* fp = NULL;
  fp = fopen(filename.c_str(), "r");
  if(!fp)
    {
      cerr << "cannot open file " << filename << endl;
      return;
    }

  meshData.clear();

  char buf[OBJ_LINE_BUF_SIZE];
  float val[6];
  int idx[256][3];
  int match;
  unsigned int type = 0;
  vector<FloatType> v,c;
  v.resize(3); c.resize(3);


  while( fscanf(fp, "%s", buf) != EOF ) {
    switch( buf[0] ){
    case '#':
      skipLine(buf, OBJ_LINE_BUF_SIZE, fp);
      break;
    case 'v':
      switch(buf[1]) {
      case '\0':
        // vertex
        val[3] = 1.0f;
        //meshlab stores colors right after vertex pos (3 xyz, 3 rgb)
        match = fscanf( fp, "%f %f %f %f %f %f",
                        &val[0], &val[1], &val[2], &val[3], &val[4], &val[5]);
        // match = fscanf( fp, "%f %f %f %f %f %f",
        //     &v[0], &v[1], &v[2], &c[0], &c[1], &c[2]);
        assert( match == 3 || match == 4 || match == 6);
        v[0] = val[0];
        v[1] = val[1];
        v[2] = val[2];
        meshData.vertices.push_back(v);
        if(match == 6){
          c[0] = val[3];
          c[1] = val[4];
          c[2] = val[5];
          meshData.colors.push_back(c);
        }
        break;

      case 'n':
        // normal
        match = fscanf( fp, "%f %f %f", &val[0], &val[1], &val[2]);
        v[0] = val[0];
        v[1] = val[1];
        v[2] = val[2];
        assert( match == 3);
        meshData.normals.push_back(v);
        break;
      }
      break;

    case 'f':
      // face
      fscanf( fp, "%s", buf);
      {
        unsigned int n= 2;
        // determine the type, and read the initial vertex
        if(sscanf(buf, "%d//%d", &idx[0][0], &idx[0][1]) == 2 ) {
          type = 4;
          // This face has vertex and normal indices

          //remap them to the right spot
          idx[0][0] = (idx[0][0] > 0) ? (idx[0][0] - 1) : ((int)meshData.vertices.size() - idx[0][0]);

          //grab the second vertex to prime
          fscanf( fp, "%d//%d", &idx[1][0], &idx[1][1]);

          //remap them to the right spot
          idx[1][0] = (idx[1][0] > 0) ? (idx[1][0] - 1) : ((int)meshData.vertices.size() - idx[1][0]);

          //create the fan
          while ( fscanf( fp, "%d//%d", &idx[n][0], &idx[n][1]) == 2) {
            //remap them to the right spot
            idx[n][0] = (idx[n][0] > 0) ? (idx[n][0] - 1) : ((int)meshData.vertices.size() - idx[n][0]);
            n++;
          }
        }
        else if (sscanf( buf, "%d/%d/%d", &idx[0][0], &idx[0][1], &idx[0][2]) == 3 ) {
          type = 3;
          //This face has vertex, texture coordinate, and normal indices

          //remap them to the right spot
          idx[0][0] = (idx[0][0] > 0) ? (idx[0][0] - 1) : ((int)meshData.vertices.size() - idx[0][0]);

          //grab the second vertex to prime
          fscanf( fp, "%d/%d/%d", &idx[1][0], &idx[1][1], &idx[1][2]);

          //remap them to the right spot
          idx[1][0] = (idx[1][0] > 0) ? (idx[1][0] - 1) : ((int)meshData.vertices.size() - idx[1][0]);

          //create the fan
          while ( fscanf( fp, "%d/%d/%d", &idx[n][0], &idx[n][1], &idx[n][2]) == 3) {
            //remap them to the right spot
            idx[n][0] = (idx[n][0] > 0) ? (idx[n][0] - 1) : ((int)meshData.vertices.size() - idx[n][0]);
            n++;
          }
        }
        else if ( sscanf( buf, "%d/%d/", &idx[0][0], &idx[0][1]) == 2) {
          type = 2;
          //This face has vertex and texture coordinate indices

          //remap them to the right spot
          idx[0][0] = (idx[0][0] > 0) ? (idx[0][0] - 1) : ((int)meshData.vertices.size() - idx[0][0]);

          //grab the second vertex to prime
          fscanf( fp, "%d/%d/", &idx[1][0], &idx[1][1]);

          //remap them to the right spot
          idx[1][0] = (idx[1][0] > 0) ? (idx[1][0] - 1) : ((int)meshData.vertices.size() - idx[1][0]);

          //create the fan
          while ( fscanf( fp, "%d/%d/", &idx[n][0], &idx[n][1]) == 2) {
            //remap them to the right spot
            idx[n][0] = (idx[n][0] > 0) ? (idx[n][0] - 1) : ((int)meshData.vertices.size() - idx[n][0]);
            n++;
          }
        }
        else if ( sscanf( buf, "%d", &idx[0][0]) == 1) {
          type = 1;
          //This face has only vertex indices

          //remap them to the right spot
          idx[0][0] = (idx[0][0] > 0) ? (idx[0][0] - 1) : ((int)meshData.vertices.size() - idx[0][0]);

          //grab the second vertex to prime
          fscanf( fp, "%d", &idx[1][0]);

          //remap them to the right spot
          idx[1][0] = (idx[1][0] > 0) ? (idx[1][0] - 1) : ((int)meshData.vertices.size() - idx[1][0]);

          //create the fan
          while ( fscanf( fp, "%d", &idx[n][0]) == 1) {
            //remap them to the right spot
            idx[n][0] = (idx[n][0] > 0) ? (idx[n][0] - 1) : ((int)meshData.vertices.size() - idx[n][0]);
            n++;
          }
        }
        else{
          cerr << filename + ": broken obj(face line invalid)";
        }

        if (n < 3)
          cerr << filename + ": broken obj (face with less than 3 indices)";

        //create face
        std::vector<unsigned int> face;
        for(unsigned int i = 0; i < n; ++i) {
          face.push_back(idx[i][0]);
        }

        if(face.size()) meshData.facesVerticesInd.push_back(face);

      }

      break;

    case 's':
    case 'g':
    case 'u':
    default:
      skipLine( buf, OBJ_LINE_BUF_SIZE, fp);

    };

  }

  fclose(fp);

  meshData.numVertices = meshData.vertices.size();
  meshData.numFaces = meshData.facesVerticesInd.size();

  if(meshData.normals.size() == 0)
    meshData.computeNormalsNeil();

}

template<class FloatType>
void MeshIO<FloatType>::updateFromPLY(const std::string& filename,
                                      MeshData<FloatType>& meshData)
{

  PlyFile *ply;
	char **elist;
	int file_type;
	float version;
	int nelems, num_elems, nprops;
	char *elem_name;
	PlyProperty **plist;
	int num_comments;
	char **comments;
	int num_obj_info;
	char **obj_info;

	/* open a PLY file for reading */
	ply = ply_open_for_reading(filename.c_str(), &nelems, &elist, &file_type, &version);

	/* go through each kind of element that we learned is in the file */
	/* and read them */

  unsigned int vertex_type;

	for (int i = 0; i < nelems; i++) {

		/* get the description of the first element */
		elem_name = elist[i];
		plist = ply_get_element_description(ply, elem_name, &num_elems, &nprops);

		/* if we're on vertex elements, read them in */
		if (equal_strings(ply::elem_names[0], elem_name)) {
			int num_vertices = num_elems;

			vertex_type = ply::get_vertex_type(plist, nprops);

			switch (vertex_type)
			{
			case PLY_VERTEX_RGB:
				ply_read_vertices<ply::VertexColor>(ply, vertex_type, num_vertices,
					meshData);
				break;
			case PLY_VERTEX_NORMAL_RGB:
				ply_read_vertices<ply::VertexNormalColor>(ply, vertex_type, num_vertices,
					meshData);
				break;
			case PLY_VERTEX_RGBA:
				ply_read_vertices<ply::VertexColorAlpha>(ply, vertex_type, num_vertices,
					meshData);
				break;
			case PLY_VERTEX_NORMAL_RGBA:
				ply_read_vertices<ply::VertexNormalColorAlpha>(ply, vertex_type, num_vertices,
					meshData);
				break;
			default:
				break;
			}
		}

	}

	/* grab and print out the comments in the file */
	comments = ply_get_comments(ply, &num_comments);

	/* grab and print out the object information */
	obj_info = ply_get_obj_info(ply, &num_obj_info);

	/* close the PLY file */
	ply_close(ply);

  if(vertex_type == PLY_VERTEX_RGBA || vertex_type == PLY_VERTEX_RGB)
    meshData.computeNormalsNeil();

}

template<class FloatType>
void MeshIO<FloatType>::updateFromOBJ(const std::string& filename,
                                      MeshData<FloatType>& meshData)
{
  // this is much faster, as we just update the data and
  // avoid using any vector push_back and don't need to
  // update the faces and colors as well, these are fixed
  // during the whole sequence
  FILE* fp = NULL;
  fp = fopen(filename.c_str(), "r");
  if(!fp)
    {
      cerr << "cannot open file " << filename << endl;
      return;
    }

  char buf[OBJ_LINE_BUF_SIZE];
  double val[6];
  int match;

  int vertex_id = 0;
  int normal_id = 0;
  FloatType* vertex_pt = NULL;
  FloatType* color_pt = NULL;
  FloatType* normal_pt = NULL;

  while( fscanf(fp, "%s", buf) != EOF ) {
    switch( buf[0] ){
    case '#':
      skipLine(buf, OBJ_LINE_BUF_SIZE, fp);
      break;
    case 'v':
      switch(buf[1]) {
      case '\0':
        vertex_pt = &meshData.vertices[vertex_id][0];
        color_pt = &meshData.colors[vertex_id][0];
        //meshlab stores colors right after vertex pos (3 xyz, 3 rgb)
        match = fscanf( fp, "%lf %lf %lf %lf %lf %lf",
                        vertex_pt , vertex_pt + 1, vertex_pt + 2,
                        &val[3], &val[4], &val[5]);
        assert( match == 3 || match == 4 || match == 6);
        if(match == 6){
          color_pt[0] = val[3];
          color_pt[1] = val[4];
          color_pt[2] = val[5];
        }
        ++vertex_id;
        break;
      case 'n':
        // normal
        normal_pt = &meshData.normals[normal_id][0];
        match = fscanf( fp, "%lf %lf %lf",
                        normal_pt, normal_pt + 1, normal_pt + 2);
        assert( match == 3);
        ++normal_id;
        break;
      }
      break;
    case 's':
    case 'g':
    case 'u':
    default:
      skipLine( buf, OBJ_LINE_BUF_SIZE, fp);

    };

  }

  fclose(fp);

  if(normal_id == 0)
    meshData.computeNormalsNeil();

}

template<class FloatType>
void MeshIO<FloatType>::writeToFile(const std::string& filename, const MeshData<FloatType>& meshData)
{
  bfs::path filePath(filename.c_str());
  if(filePath.extension().compare(std::string(".off")) == 0) {
    writeToOFF(filename,meshData);
  } else if(filePath.extension().compare(std::string(".ply")) == 0){
    writeToPLY(filename,meshData);
  } else if(filePath.extension().compare(std::string(".obj")) == 0){
    writeToOBJ(filename,meshData);
  } else {
    cerr << "file format not supported for mesh saving" << endl;
  }

}

template<class FloatType>
void MeshIO<FloatType>::writeToOBJ(const std::string& filename,
                                   const MeshData<FloatType>& meshData)
{
  std::ofstream file(filename.c_str());
	if (!file.is_open())
    cerr << "could not open file for writing " + filename << endl;

  file << "####\n";
	file << "#\n";
	file << "# OBJ file Generated by Rui\n";
	file << "#\n";
	file << "####\n";
	file << "# Object " << filename << "\n";
	file << "#\n";
	file << "# Vertices: " << meshData.numVertices << "\n";
	file << "# Faces: " << meshData.numFaces << "\n";
	file << "#\n";
	file << "####\n";

  for (int i = 0; i < meshData.vertices.size(); i++) {
		file << "v ";
		file << meshData.vertices[i][0] << " " << meshData.vertices[i][1] << " " << meshData.vertices[i][2];
		if (meshData.colors.size() > 0) {
			file << " " << meshData.colors[i][0] << " " << meshData.colors[i][1] << " " << meshData.colors[i][2];
		}
		file << "\n";
	}

  for(int i = 0; i < meshData.normals.size(); ++i)
    {
      file << "vn ";
      file << meshData.normals[i][0] << " " << meshData.normals[i][1] << " " << meshData.normals[i][2] << "\n";
    }

  for(int i = 0; i < meshData.facesVerticesInd.size(); ++i)
    {
      file << "f ";
      // for triangle mesh
      for(int j = 0; j < 3; ++j){
        file << meshData.facesVerticesInd[i][j] + 1;
        file << " ";
      }
      file << "\n";
    }

  file.close();

}

template<class FloatType>
void MeshIO<FloatType>::writeToPLY(const std::string& filename, const MeshData<FloatType>& meshData)
{
	PlyFile *ply;
	float version;

	/* open either a binary or ascii PLY file for writing */
	/* (the file will be called "test.ply" because the routines */
	/*  enforce the .ply filename extension) */

#ifdef PLY_SAVE_ASCII
	ply = ply_open_for_writing(filename.c_str(), 2, ply::elem_names, PLY_ASCII, &version);
#else
	ply = ply_open_for_writing(filename.c_str(), 2, ply::elem_names, PLY_BINARY_LE, &version);
#endif

	/* describe what properties go into the vertex and face elements */

	int num_vertices = meshData.numVertices;
	ply_element_count(ply, ply::elem_names[0], num_vertices);
	int vertex_type = 1;
	for (int i = 0; i < ply::n_vprops[vertex_type]; i++)
	{
		PlyProperty* prop = ply::get_vertex_property(vertex_type, i);
		ply_describe_property(ply, ply::elem_names[0], prop);
	}

	int num_faces = meshData.numFaces;
	ply_element_count(ply, ply::elem_names[1], num_faces);
	for (int i = 0; i < ply::n_fprops; i++)
	{
		ply_describe_property(ply, ply::elem_names[1], &ply::face_props[i]);
	}

	/* we have described exactly what we will put in the file, so */
	/* we are now done with the header info */
	ply_header_complete(ply);

	/* set up and write the vertex elements */
	ply_put_element_setup(ply, ply::elem_names[0]);
	//for (i = 0; i < nverts; i++)
	for (int i = 0; i < num_vertices; i++){
		ply::VertexNormalColor v;
		v.x = meshData.vertices[i][0];
		v.y = meshData.vertices[i][1];
		v.z = meshData.vertices[i][2];
		v.nx = meshData.normals[i][0];
		v.ny = meshData.normals[i][1];
		v.nz = meshData.normals[i][2];
		v.r = (unsigned char)(meshData.colors[i][0] * 255.f);
		v.g = (unsigned char)(meshData.colors[i][1] * 255.f);
		v.b = (unsigned char)(meshData.colors[i][2] * 255.f);
		ply_put_element(ply, (void *)&v);
	}

	/* set up and write the face elements */
	ply_put_element_setup(ply, ply::elem_names[1]);
	ply::Face f;
	f.verts = new int[10];
	for (int i = 0; i < num_faces; i++)
	{
		f.nverts = (unsigned char)meshData.facesVerticesInd[i].size();
		for (int j = 0; j < f.nverts; j++)
		{
			f.verts[j] = meshData.facesVerticesInd[i][j];
		}
		ply_put_element(ply, (void *)&f);
	}

	/* close the PLY file */
	ply_close(ply);
}

template<class FloatType>
void MeshIO<FloatType>::createMeshFromDepth(MeshData<FloatType>& meshData,
                                            unsigned char* pColorImageRGB, DepthImageType& uImage,
                                            DepthImageType vImage, DepthImageType& dImage,
                                            InternalIntensityImageType& maskImage, int rows, int cols, double scale)
{
  int m_nHeight, m_nWidth;
  m_nHeight = rows * scale;
  m_nWidth = cols * scale;

  // compute normals first
  CoordinateType* pResults = new CoordinateType[3*m_nHeight*m_nWidth];
  CoordinateType* pNormals = new CoordinateType[3*m_nHeight*m_nWidth];
  CoordinateType* pColors = new CoordinateType[3*m_nHeight*m_nWidth];

  //set up pResults first and mask vector
  vector<double> maskVector;
  vector<unsigned int> modelLabels;
  vector<FloatType> modelColors;
  int offset;
  int imageOffset;
  int imageI, imageJ;
  for(int i = 0; i < m_nHeight; ++i)
    {
      for(int j = 0; j < m_nWidth; ++j)
        {
          offset = 3*(i*m_nWidth+j);
          imageI = i/scale;
          imageJ = j/scale;
          imageOffset = 3*(imageI*m_nWidth/scale + imageJ);
          pResults[offset] = uImage(imageI,imageJ);
          pResults[offset+1] = vImage(imageI,imageJ);
          pResults[offset+2] = dImage(imageI,imageJ);
          pColors[offset] = pColorImageRGB[imageOffset] / 255.0;
          pColors[offset+1] = pColorImageRGB[imageOffset+1] / 255.0;
          pColors[offset+2] = pColorImageRGB[imageOffset+2] / 255.0;
          maskVector.push_back(maskImage(imageI,imageJ));
          modelLabels.push_back(0);
          modelColors.push_back(1);
          modelColors.push_back(0);
          modelColors.push_back(0);

        }
    }

  unsigned int radius = 1;

  for(int j = 0; j < m_nHeight - radius; ++j)
    {
      for(int i = 0; i < m_nWidth - radius; ++ i)
        compnorm(&pResults[0]+3*(j*m_nWidth+i),&pResults[0]+3*(j*m_nWidth+radius+i),
                 &pResults[0]+3*((j+radius)*m_nWidth+i),&pNormals[0]+3*(j*m_nWidth+i), true);

      for(int i = m_nWidth-radius; i !=m_nWidth; ++i)
        compnorm(&pResults[0]+3*(j*m_nWidth+i-radius), &pResults[0]+3*(j*m_nWidth+i),
                 &pResults[0]+3*((j+radius)*m_nWidth+i),&pNormals[0]+3*(j*m_nWidth+i), true);
    }

  for (int j = m_nHeight-radius; j < m_nHeight; ++j)
    {
      for(int i = 0; i < m_nWidth-radius; ++i)
        compnorm(&pResults[0]+3*((j-radius)*m_nWidth+i), &pResults[0]+3*(j*m_nWidth+radius+i),
                 &pResults[0]+3*(j*m_nWidth+i), &pNormals[0]+3*(j*m_nWidth+i), true);

      for(int i = m_nWidth-radius; i !=m_nWidth; ++i)
        compnorm(&pResults[0]+3*(j*m_nWidth+i), &pResults[0]+3*(j*m_nWidth-radius+i),
                 &pResults[0]+3*((j-radius)*m_nWidth+i), &pNormals[0]+3*(j*m_nWidth+i), true);
    }

  createMeshFromPoints(meshData, m_nHeight, m_nWidth, pResults, pNormals,
                       pColors, maskVector, modelLabels, modelColors);

  meshData.clockwise = true;

  delete[] pColors;
  delete[] pNormals;
  delete[] pResults;

}

template<class FloatType>
void MeshIO<FloatType>::createMeshFromDepth(MeshData<FloatType>& meshData,
                                            InternalColorImageType& colorImage, DepthImageType& uImage,
                                            DepthImageType vImage, DepthImageType& dImage,
                                            InternalIntensityImageType& maskImage, int rows, int cols, double scale)
{

  int m_nHeight, m_nWidth;
  m_nHeight = rows * scale;
  m_nWidth = cols * scale;

  // compute normals first
  CoordinateType* pResults = new CoordinateType[3*m_nHeight*m_nWidth];
  CoordinateType* pNormals = new CoordinateType[3*m_nHeight*m_nWidth];
  CoordinateType* pColors = new CoordinateType[3*m_nHeight*m_nWidth];

  //set up pResults first and mask vector
  vector<double> maskVector;
  vector<unsigned int> modelLabels;
  vector<FloatType> modelColors;
  int offset;
  int imageOffset;
  int imageI, imageJ;
  for(int i = 0; i < m_nHeight; ++i)
    {
      for(int j = 0; j < m_nWidth; ++j)
        {
          offset = 3*(i*m_nWidth+j);
          imageI = i/scale;
          imageJ = j/scale;
          imageOffset = 3*(imageI*m_nWidth/scale + imageJ);
          pResults[offset] = uImage(imageI,imageJ);
          pResults[offset+1] = vImage(imageI,imageJ);
          pResults[offset+2] = dImage(imageI,imageJ);

          pColors[offset] =   colorImage.at<Vec3d>(imageI, imageJ)[0];
          pColors[offset+1] = colorImage.at<Vec3d>(imageI, imageJ)[1];
          pColors[offset+2] = colorImage.at<Vec3d>(imageI, imageJ)[2];

          maskVector.push_back(maskImage(imageI,imageJ));
          modelLabels.push_back(0);
          modelColors.push_back(1);
          modelColors.push_back(0);
          modelColors.push_back(0);

        }
    }

  unsigned int radius = 1;

  for(int j = 0; j < m_nHeight - radius; ++j)
    {
      for(int i = 0; i < m_nWidth - radius; ++ i)
        compnorm(&pResults[0]+3*(j*m_nWidth+i),&pResults[0]+3*(j*m_nWidth+radius+i),
                 &pResults[0]+3*((j+radius)*m_nWidth+i),&pNormals[0]+3*(j*m_nWidth+i), true);

      for(int i = m_nWidth-radius; i !=m_nWidth; ++i)
        compnorm(&pResults[0]+3*(j*m_nWidth+i-radius), &pResults[0]+3*(j*m_nWidth+i),
                 &pResults[0]+3*((j+radius)*m_nWidth+i),&pNormals[0]+3*(j*m_nWidth+i), true);
    }

  for (int j = m_nHeight-radius; j < m_nHeight; ++j)
    {
      for(int i = 0; i < m_nWidth-radius; ++i)
        compnorm(&pResults[0]+3*((j-radius)*m_nWidth+i), &pResults[0]+3*(j*m_nWidth+radius+i),
                 &pResults[0]+3*(j*m_nWidth+i), &pNormals[0]+3*(j*m_nWidth+i), true);

      for(int i = m_nWidth-radius; i !=m_nWidth; ++i)
        compnorm(&pResults[0]+3*(j*m_nWidth+i), &pResults[0]+3*(j*m_nWidth-radius+i),
                 &pResults[0]+3*((j-radius)*m_nWidth+i), &pNormals[0]+3*(j*m_nWidth+i), true);
    }

  createMeshFromPoints(meshData, m_nHeight, m_nWidth, pResults, pNormals,
                       pColors, maskVector, modelLabels, modelColors);

  meshData.clockwise = true;

  delete[] pColors;
  delete[] pNormals;
  delete[] pResults;


}

template<class FloatType>
void MeshIO<FloatType>::createMeshFromPoints(MeshData<FloatType>& meshData, int rows, int cols,
                                             FloatType* pPointsImage, FloatType* pNormalsImage, FloatType* pColorsImage,
                                             vector<double>& maskImage,  vector<unsigned int>& modelLabels,
                                             vector<FloatType>& modelColors)
{
  vector<FloatType> Points3d, Normals3d, Colors3d;
  Points3d.resize(3);
  Normals3d.resize(3);
  Colors3d.resize(3);

  int modelNum = modelColors.size()/3;
  for(int i = 0; i < modelNum; ++i)
    {
      Colors3d[0] = modelColors[3*i];
      Colors3d[1] = modelColors[3*i+1];
      Colors3d[2] = modelColors[3*i+2];
      meshData.modelColors.push_back(Colors3d);
    }

  vector<int> pPntsInd(rows*cols, -1);
  // row major ordering of the points we have
  int pntsId= 0;
  double center[3] = {0,0,0};
  FloatType gray;

  ColorImageType colorImageBGR; // opencv default order, BGR
  IntensityImageType grayImageBYTE;
  InternalIntensityImageType grayImage;
  unsigned char* pCurrentColorImageBGR = new unsigned char[3*rows*cols];
  for(int i = 0; i < rows*cols; ++i)
    {
      pCurrentColorImageBGR[ 3*i ] = pColorsImage[ 3*i + 2 ] * 255;
      pCurrentColorImageBGR[ 3*i + 1] = pColorsImage[ 3*i + 1 ] * 255;
      pCurrentColorImageBGR[ 3*i + 2 ] = pColorsImage[ 3*i ] * 255;
    }
  cv::Mat tempColorImage(rows, cols, CV_8UC3, pCurrentColorImageBGR);
  colorImageBGR = tempColorImage;
  cv::cvtColor( colorImageBGR, grayImageBYTE, CV_BGR2GRAY );
  grayImageBYTE.convertTo(grayImage, cv::DataType<CoordinateType>::type, 1./255);

  delete[] pCurrentColorImageBGR;

  for(int i = 0; i < rows; ++i)
    {
      for(int j = 0; j < cols; ++j)
        {
          int k = i*cols + j;
          if(maskImage[k])
            {
              meshData.grays.push_back(grayImage(i,j));
              // if(maskImage[k] > 0 && maskImage[k] < 255)
              // std::cout << maskImage[k] << std::endl;
              for(int ind = 0; ind < 3; ++ind)
                {
                  Points3d[ind] = pPointsImage[ 3*k + ind ];
                  Normals3d[ind] = pNormalsImage[ 3*k + ind ];
                  Colors3d[ind] = pColorsImage[ 3*k + ind ];
                  center[ind] += pPointsImage[ 3*k + ind];
                }

              meshData.vertices.push_back(Points3d);
              meshData.normals.push_back(Normals3d);
              meshData.colors.push_back(Colors3d);
              meshData.modelLabels.push_back(modelLabels[k]);
              // gray = 0.299*Colors3d[0] + 0.587*Colors3d[1] + 0.114*Colors3d[2];
              // meshData.grays.push_back(gray);
              pPntsInd[k] = pntsId;
              ++pntsId;
            }
        }
    }

  // for(int k = 0; k < rows*cols; ++k)
  // {
  //     if(maskImage[k])
  //     {
  //         // if(maskImage[k] > 0 && maskImage[k] < 255)
  //         // std::cout << maskImage[k] << std::endl;
  //         for(int ind = 0; ind < 3; ++ind)
  //         {
  //             Points3d[ind] = pPointsImage[ 3*k + ind ];
  //             Normals3d[ind] = pNormalsImage[ 3*k + ind ];
  //             Colors3d[ind] = pColorsImage[ 3*k + ind ];
  //             center[ind] += pPointsImage[ 3*k + ind];
  //         }

  //         meshData.vertices.push_back(Points3d);
  //         meshData.normals.push_back(Normals3d);
  //         meshData.colors.push_back(Colors3d);
  //         meshData.modelLabels.push_back(modelLabels[k]);
  //         // gray = 0.299*Colors3d[0] + 0.587*Colors3d[1] + 0.114*Colors3d[2];
  //         // meshData.grays.push_back(gray);
  //         pPntsInd[k] = pntsId;
  //         ++pntsId;
  //     }
  //     // }
  // }

  // calculate the center value of the mesh
  for(int ind = 0; ind < 3; ++ind)
    meshData.center[ind] = center[ind] / pntsId;

  meshData.numVertices = meshData.vertices.size();
  // now we know the index of all the vertices, we can create the faces
  std::vector<unsigned int> triVerticesInd;
  triVerticesInd.resize(3);
  int self, upper, lower, left, right;
  meshData.adjVerticesInd.resize(meshData.numVertices);
  for(int i = 0; i < rows; ++i)
    {
      for(int j=0; j < cols; ++j)
        {
          self = i*cols + j;
          if(maskImage[i*cols+j])
            {
              upper = (i-1)*cols + j;
              lower = (i+1)*cols + j;
              left  = i*cols + j - 1;
              right = i*cols + j + 1;
              // for p(i,j), add p(i-1,j), p(i+1,j), p(i,j-1), p(i,j+1)
              // triangle (i,j), (i,j-1), (i-1,j)
              if( i > 0 && maskImage[upper] &&
                  j > 0 && maskImage[left])
                {
                  triVerticesInd[0] = pPntsInd[self];
                  triVerticesInd[1] = pPntsInd[left];
                  triVerticesInd[2] = pPntsInd[upper];
                  meshData.facesVerticesInd.push_back(triVerticesInd);
                }
              // triangle (i,j), (i-1,j), (i,j+1)
              if( i > 0 && maskImage[upper] &&
                  j < cols-1 && maskImage[right])
                {
                  triVerticesInd[0] = pPntsInd[self];
                  triVerticesInd[1] = pPntsInd[upper];
                  triVerticesInd[2] = pPntsInd[right];
                  meshData.facesVerticesInd.push_back(triVerticesInd);
                }
              // triangle (i,j), (i+1,j), (i,j-1)
              if(i < rows && maskImage[lower] &&
                 j > 0 && maskImage[left])
                {
                  triVerticesInd[0] = pPntsInd[self];
                  triVerticesInd[1] = pPntsInd[lower];
                  triVerticesInd[2] = pPntsInd[left];
                  meshData.facesVerticesInd.push_back(triVerticesInd);
                }
              // triangle (i,j), (i,j+1), (i+1,j)
              if(i < rows && maskImage[lower] &&
                 j < cols && maskImage[right])
                {
                  triVerticesInd[0] = pPntsInd[self];
                  triVerticesInd[1] = pPntsInd[right];
                  triVerticesInd[2] = pPntsInd[lower];
                  meshData.facesVerticesInd.push_back(triVerticesInd);
                }
              // add all possible neighbors, in this case, some edges may not necessarily
              // belong to an edge, save the neighbors in a clock-wise order, keep this
              // in mind when compute the normals
              if( i > 0 && maskImage[upper] )
                meshData.adjVerticesInd[ pPntsInd[ self ] ].push_back( pPntsInd[upper] );
              if( j < cols && maskImage[right] )
                meshData.adjVerticesInd[ pPntsInd[ self ] ].push_back( pPntsInd[right] );
              if( i < rows && maskImage[lower] )
                meshData.adjVerticesInd[ pPntsInd[ self ] ].push_back( pPntsInd[lower] );
              if( j > 0 && maskImage[left] )
                meshData.adjVerticesInd[ pPntsInd[ self ] ].push_back( pPntsInd[left] );
            }
        }
    }

  meshData.numFaces = meshData.facesVerticesInd.size();
  // // add neighborhood vertices
  // meshData.adjVerticesInd.resize(meshData.numVertices);
  // for(int face = 0; face < meshData.numFaces; ++face)
  // {
  //     int vertexNum = meshData.facesVerticesInd[face].size();
  //     for(int ind = 0; ind < vertexNum; ++ind)
  //     {
  //         for(int offset = 1; offset < vertexNum; ++offset)
  //         {
  //             meshData.adjVerticesInd[ meshData.facesVerticesInd[face][ind] ].push_back( meshData.facesVerticesInd[face]
  //                     [ ind + offset >= vertexNum ? ind + offset - vertexNum : ind + offset ] );
  //         }
  //     }
  // }

  // // remove redundant vertices index.
  // for(int vertex=0; vertex < meshData.numVertices; ++vertex)
  // {
  //     vector<unsigned int>& adjVertex = meshData.adjVerticesInd[vertex];
  //     std::sort( adjVertex.begin(), adjVertex.end() );
  //     vector<unsigned int>::iterator end_unique = std::unique( adjVertex.begin(), adjVertex.end() );
  //     adjVertex.erase(end_unique, adjVertex.end());
  // }
}

template<class FloatType>
void MeshIO<FloatType>::updateMesh(MeshData<FloatType>& meshData, int rows, int cols,
                                   FloatType* pPointsImage, FloatType* pNormalsImage, FloatType* pColorsImage,
                                   vector<double>& maskImage,  vector<unsigned int>& modelLabels,
                                   vector<FloatType>& modelColors)
{
  // row major ordering of the points we have
  int pntsId= 0;
  double center[3] = {0,0,0};
  for(int k = 0; k < rows*cols; ++k)
    {
      if(maskImage[k])
        {
          for(int ind = 0; ind < 3; ++ind)
            {
              meshData.vertices[pntsId][ind] = pPointsImage[ 3*k + ind ];
              meshData.normals[pntsId][ind] = pNormalsImage[ 3*k + ind ];
            }
          ++pntsId;
        }
    }
}

template<class FloatType>
void MeshIO<FloatType>::setupMeshFromFile(MeshData<FloatType>& meshData)
{
  // set up numVertices, numFaces, gray values, modelColors, and etc
  // the adjacent edges information for the mesh
  // meshData.numVertices = meshData.vertices.size();
  // meshData.numFaces = meshData.facesVerticesInd.size();

  if(meshData.colors.size() > 0)
    {
      meshData.grays.resize(meshData.numVertices);
      for(int i = 0; i < meshData.numVertices; ++i)
        {
          // color2gray
          meshData.grays[i] = 0.299*meshData.colors[i][0] +
            0.587*meshData.colors[i][1] +
            0.114*meshData.colors[i][2];
        }
    }

  FloatType center[3] = {0, 0, 0};
  for(int i = 0; i < meshData.numVertices; ++i)
    {
      center[0] = center[0] + meshData.vertices[i][0];
      center[1] = center[1] + meshData.vertices[i][1];
      center[2] = center[2] + meshData.vertices[i][2];
    }

  for(int i = 0; i < 3; ++i)
    meshData.center[i] = center[i]/meshData.numVertices;

  meshData.adjVerticesInd.resize(meshData.numVertices);
  for(int i = 0; i < meshData.numFaces; ++i)
    {
      meshData.adjVerticesInd[ meshData.facesVerticesInd[i][0] ].
        push_back( meshData.facesVerticesInd[i][1] );
      meshData.adjVerticesInd[ meshData.facesVerticesInd[i][0] ].
        push_back( meshData.facesVerticesInd[i][2] );
      meshData.adjVerticesInd[ meshData.facesVerticesInd[i][1] ].
        push_back( meshData.facesVerticesInd[i][0] );
      meshData.adjVerticesInd[ meshData.facesVerticesInd[i][1] ].
        push_back( meshData.facesVerticesInd[i][2] );
      meshData.adjVerticesInd[ meshData.facesVerticesInd[i][2] ].
        push_back( meshData.facesVerticesInd[i][0] );
      meshData.adjVerticesInd[ meshData.facesVerticesInd[i][2] ].
        push_back( meshData.facesVerticesInd[i][1] );
    }

  // remove duplicate neighbors
  for(int i = 0; i < meshData.numVertices; ++i)
    {
      set<unsigned int> s(meshData.adjVerticesInd[i].begin(),
                          meshData.adjVerticesInd[i].end());
      meshData.adjVerticesInd[i].assign(s.begin(), s.end());
    }

  meshData.modelLabels.resize(meshData.numVertices,1);

  vector<FloatType> labelColor;
  labelColor.resize(3);
  labelColor[0] = 1;
  labelColor[1] = 0;
  labelColor[2] = 0;

  meshData.modelColors.resize(meshData.numVertices);
  for(int i = 0; i < meshData.numVertices; ++i)
    meshData.modelColors[i] = labelColor;

  // // normals
  // if(meshData.normals.size() != meshData.numVertices) // if there is no normals
  //   {
  //     meshData.normals.resize(meshData.numVertices);
  //     for(int i = 0; i < meshData.numVertices; ++i)
  //       meshData.normals[i].resize(3);
  //     meshData.computeNormalsNeil();
  //   }

}

template<class FloatType>
void MeshIO<FloatType>::skipLine(char * buf, int size, FILE * fp)
{
  do {
    buf[size-1] = '$';
    fgets(buf, size, fp);
  } while (buf[size-1] != '$');
}

typedef MeshIO<CoordinateType> PangaeaMeshIO;
