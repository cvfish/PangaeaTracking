#include "Mesh.h"
#include "global.h"
#include "residual.h"

#include "global.cpp"
#include "Mesh.cpp"

#include "CCamera.cpp"
#include "DepthBuffer.cpp"
#include "NcvGlXContext.cpp"
#include "OptimizationStrategy.cpp"

class ParameterReader
{
public:
    ParameterReader( string filename="./parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                continue;
            }
 
            int pos = str.find("=");
            if (pos == -1)
            continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;
 
            if ( !fin.good() )
            break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};

int main(int argc, char** args)
{
    // check the range of the values in this mesh
    ParameterReader pd;
    bool use_mesh_file = atof(pd.getData("use_mesh_file").c_str());

    bool clockwise = true;
    if(pd.getData("clockwise").compare("NOT_FOUND") != 0)
    clockwise = atof(pd.getData("clockwise").c_str());

    if(use_mesh_file)
    {
        // this works for a single mesh
        PangaeaMeshData inputMesh;

        std::string input_file = pd.getData("input_mesh_file");
        std::string output_file = pd.getData("output_mesh_file");

        PangaeaMeshIO::loadfromFile(input_file, inputMesh, clockwise);
        vector<vector<double> > bbox;
        getMeshBoundingBox(inputMesh, bbox);
        
        double xrange = bbox[0][1] - bbox[0][0];
        double yrange = bbox[1][1] - bbox[1][0];
        double zrange = bbox[2][1] - bbox[2][0];

        cout << "x_range: " << xrange << endl;
        cout << "y_range: " << yrange << endl;
        cout << "z_range: " << zrange << endl;

        double factor = 400.0 / zrange;
        cout << "scaling factor is: " << factor << endl;

        // scale the mesh up

        scaleMeshUp(inputMesh, factor);
        PangaeaMeshIO::writeToFile(output_file, inputMesh);
        
    }else
    {
        // this works for a heirachy of meshes
        PangaeaMeshData inputMesh;

        std::string input_format = pd.getData("input_mesh_format");
        std::string output_format = pd.getData("output_mesh_format");
        std::string input_list_str = pd.getData("input_list");

        std::stringstream input_list(input_list_str);
        int number; vector<int> input_list_vector;

        input_list >> number;
        while( !input_list.fail() )
        {
            input_list_vector.push_back(number);
            input_list >> number;
        }

        double factor = 0;
        char buffer[BUFFER_SIZE];
        for(int i = 0; i < input_list_vector.size(); ++i)
        {
            stringstream input_file;
            sprintf(buffer, input_format.c_str(), input_list_vector[i]);
            input_file << buffer;
            //memset(&buffer[0], 0, sizeof(buffer));

            stringstream output_file;
            sprintf(buffer, output_format.c_str(), input_list_vector[i]);
            output_file << buffer;
            //memset(&buffer[0], 0, sizeof(buffer));

            PangaeaMeshIO::loadfromFile(input_file.str(), inputMesh, clockwise);

            if(factor == 0) // if this is the first frame
            {
                vector<vector<double> > bbox;
                getMeshBoundingBox(inputMesh, bbox);
                double xrange = bbox[0][1] - bbox[0][0];
                double yrange = bbox[1][1] - bbox[1][0];
                double zrange = bbox[2][1] - bbox[2][0];
                cout << "x_range: " << xrange << endl;
                cout << "y_range: " << yrange << endl;
                cout << "z_range: " << zrange << endl;
                factor = 400.0 / zrange;
                cout << "scaling factor is: " << factor << endl;
            }
            
            // scale the mesh up
            scaleMeshUp(inputMesh, factor);
            PangaeaMeshIO::writeToFile(output_file.str(), inputMesh);
        }
        
    }


}