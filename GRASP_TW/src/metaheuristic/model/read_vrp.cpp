#include "./read_vrp.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <string>
#include <cctype>
#include <unordered_map>
#include <boost/algorithm/string.hpp>


using namespace std;

template <typename T>
vector<unsigned int> sort_indexes(const vector<T> &v) {

  // initialize original index locations
  vector<unsigned int> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values 
  stable_sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}



//Sort distance matrix for Giant tour
vector<vector<unsigned int>> compute_sorted_distance_matrix(vector<vector<float>>& dist_matrix){
	vector<vector<unsigned int>> sorted_distance_matrix(dist_matrix.size());
	for(int i = 0; i < dist_matrix.size(); i++){
		sorted_distance_matrix[i] = sort_indexes(dist_matrix[i]);
	}
	
	return sorted_distance_matrix;
}


// This function uses boost library downloaded from: https://www.boost.org/users/history/version_1_73_0.html
vector<string> read_matrix_csv(const string& filename) {
    string line;
    vector<string> elements;
    vector<string> result;
    ifstream f{filename};
	
	if(!f){
		cout << "File not found" << endl;
	}else{
		while(!f.eof()){
			getline(f, line, ',');
			boost::split(result, line, boost::is_any_of(" \r\n \n"));
			for(int i = 0; i < result.size(); i++){
				if(result[i].size() != 0){
					elements.push_back(result[i]);
				}
			}
		}
	}
	
	f.close();

    return elements;
}

vector<string> read_elem(const string& filename) {
    string line;
    vector<string> elements;
    vector<string> result;
    ifstream f{filename};

    if(!f){
        cout << "File not found" << endl;
    }else{
        while(!f.eof()){
            getline(f, line);
            boost::split(result, line, [](unsigned char x){return std::isspace(x);});
            for(int i = 0; i < result.size(); i++){
                if(result[i].size() != 0){
                    elements.push_back(result[i]);
                }
            }
        }
    }

    f.close();

    return elements;
}




TSPInstanceValues read_input_tsptw(const string& filename, const int& in_test_phase) {
    cout << filename << endl;
    vector<string> elements = read_elem(filename);
    vector<string>::iterator ptr = elements.begin();

    int n_cust = 0;
    int nb_nodes = 0;
    int depot_pos = 0;
    string token;
    string route_id;
    //string quality_score = "High";


    nb_nodes = stoi(*next(ptr, 1));

    advance(ptr, 1);

    token = *next(ptr, 1);
    advance(ptr, 1);
    if (token != "vehicle_capacity") {
        cout << "Expected token vehicle_capacity but got: " << token << endl;
        exit(1);
    }

    float Q = 0;
    Q = stof(*next(ptr, 1));
    advance(ptr, 1);

    token = *next(ptr, 1);
    advance(ptr, 1);
    if (token != "depot_number") {
        cout << "Expected token depot_number" << endl;
        exit(1);
    }
    depot_pos = stoi(*next(ptr, 1)) - 1; //Check if that 1 really has to be substracted or not
    advance(ptr, 1);

    token = *next(ptr, 1);
    advance(ptr, 1);
    if (token != "route_id") {
        cout << "Expected token route_id" << endl;
        exit(1);
    }
    route_id = *next(ptr, 1);
    advance(ptr, 1);
    /*
    if (in_test_phase == 0) {
        token = *next(ptr, 1);
        advance(ptr, 1);
        if (token != "quality_score") {
            cout << "Expected token quality_score" << endl;
            exit(1);
        }
        quality_score = *next(ptr, 1);
        advance(ptr, 1);
    }
    */

    n_cust = nb_nodes;
    vector<vector<float>> time_matrix(nb_nodes, vector<float>(nb_nodes, 0));
    vector<vector<float>> time_windows(nb_nodes, vector<float>(2, 0));
    vector<vector<float>> location(nb_nodes, vector<float>(2, 0));
    vector<float> service_time(nb_nodes, 0);
    vector<float> demands(nb_nodes, 0);
    vector<string> zone_id(nb_nodes, "0");
    unordered_map<int, string> customers_map;
    unordered_map<string, int> zones_map;
    map<string, int> ordered_zones_map;

    int depot_x = 0;
    int depot_y = 0;
    int node_id;

    token = *next(ptr, 1);
    advance(ptr, 1);
    if(token != "travel_times"){
        cout << "Expected token travel_times" << endl;
        exit(1);
    }

    for(int i = 0; i < nb_nodes; i++){
        if (i == 0){
            for(int k = 0; k < nb_nodes+1; k++){
                token = *next(ptr, 1);
                advance(ptr, 1);
            }
        }
        token = *next(ptr, 1);
        customers_map[i] = token;
        advance(ptr, 1);
        for(int j = 0; j < nb_nodes; j++){
            time_matrix[i][j] = stof(*next(ptr, 1));
            advance(ptr, 1);
        }
    }

    // Compute sorted time matrix
    vector<vector<unsigned int>> sorted_time_matrix = compute_sorted_distance_matrix(time_matrix);

    //Time windows matrix
    token = *next(ptr, 1);

    advance(ptr, 1);
    if(token != "time_windows"){
        cout << "Expected token time_windows" << endl;
        exit(1);
    }
    for(int i = 0; i < nb_nodes; i++){
        for(int j = 0; j < 2; j++){
            time_windows[i][j] = stof(*next(ptr, 1));
            advance(ptr, 1);
        }
    }

    //Service time vector
    token = *next(ptr, 1);
    //cout << token << " ";
    advance(ptr, 1);
    if(token != "service_time"){
        cout << "Expected token service_time" << endl;
        exit(1);
    }
    for(int i = 0; i < nb_nodes; i++){
            service_time[i] = stof(*next(ptr, 1));
            advance(ptr, 1);
    }

    //Demands vector
    token = *next(ptr, 1);
    advance(ptr, 1);
    if(token != "dimensions"){
        cout << "Expected token dimensions" << endl;
        exit(1);
    }
    for(int i = 0; i < nb_nodes; i++){
        demands[i] = stof(*next(ptr, 1));
        advance(ptr, 1);
    }

    //Latitude Longitude
    token = *next(ptr, 1);
    advance(ptr, 1);
    if(token != "latitude_longitude"){
        cout << "Expected token latitude_longitude" << endl;
        exit(1);
    }
    for(int i = 0; i < nb_nodes; i++){
        for(int j = 0; j < 2; j++){
            location[i][j] = stof(*next(ptr, 1));
            advance(ptr, 1);
        }
    }

    token = *next(ptr, 1);
    advance(ptr, 1);
    if(token != "zone_id"){
        cout << "Expected token zone_id" << endl;
        exit(1);
    }
    for(int i = 0; i < nb_nodes; i++){
        token = *next(ptr, 1);
        zone_id[i] = token;
        // check if key 'token' exists in the map or not
        std::unordered_map<string, int>::iterator it = zones_map.find(token);

        // key already present in the map
        if (it != zones_map.end()) {
            it->second++;    // increment map's value for key 'token'
        }
            // key not found
        else {
            zones_map.insert(std::make_pair(token, 1));
        }
        advance(ptr, 1);
    }

    for (auto zone : zones_map ){
        ordered_zones_map.insert(pair<string, int>(zone.first, zone.second));
    }

    //vector<vector<vector<float>>> insertion_tsp_matrix;


    cout << "Parsed file correctly" << endl;

    return {n_cust, depot_pos, Q, time_matrix, time_windows, service_time, demands, location, sorted_time_matrix, customers_map, zone_id, zones_map, ordered_zones_map};
}

TSPHistoricValues read_historic_files(const string& route_fid, const int& number_of_zones, const int& number_of_customers, const int& in_test_phase) {
    string prob_matrix_file( "../prob_matrices_heading/"+route_fid+"_probs.csv" );
    string angles_matrix_file( "../prob_matrices_angle/"+route_fid+"_probs.csv" );
    string historic_zones_file( "../prob_matrices_heading/"+route_fid+"_zroutes.csv" );


    if (in_test_phase == 1){
        prob_matrix_file =  "../data/model_apply_outputs/prob_matrices_heading/"+route_fid+"_probs.csv" ;
        angles_matrix_file =  "../data/model_apply_outputs/prob_matrices_angle/"+route_fid+"_probs.csv" ;
        historic_zones_file = "../data/model_apply_outputs/prob_matrices_heading_2/"+route_fid+"_zroutes.csv" ;
    }
    else {
        prob_matrix_file =  "../data/model_build_outputs/prob_matrices_heading/"+route_fid+"_probs.csv" ;
        angles_matrix_file =  "../data/model_build_outputs/prob_matrices_angle/"+route_fid+"_probs.csv" ;
        historic_zones_file = "../data/model_build_outputs/prob_matrices_heading_2/"+route_fid+"_zroutes.csv" ;
    }

    cout << " In read historic files, route_fid: " << route_fid<< endl;

    vector<string> elements = read_matrix_csv(prob_matrix_file);
    vector<string>::iterator ptr = elements.begin();
    vector<string> elements_3 = read_matrix_csv(historic_zones_file);
    vector<string>::iterator ptr_3 = elements_3.begin();
    vector<string> elements_4 = read_matrix_csv(angles_matrix_file); \
    vector<string>::iterator ptr_4 = elements_4.begin();



    string token;
    string route_id;
    string zone_1;
    string zone_2;
    string zone_3;
    int nb_nodes = number_of_zones -1;
    int max_n_routes = 1;


    vector<vector<float>> probabilities_matrix(nb_nodes+1, vector<float>(nb_nodes+1, 0));
    vector<vector<float>> angles_matrix(nb_nodes+1, vector<float>(nb_nodes+1, 0));
    vector<vector<int>> n_routes_matrix(nb_nodes+1, vector<int>(nb_nodes+1, 0));
    vector<string> historic_zones_id(nb_nodes+1, "0");
    vector<string> correct_zones_id(nb_nodes+1, "0");
    cout << "Size matrix: " << nb_nodes << endl;

    unordered_map< string, unordered_map<string, float> > probabilities_df;
    unordered_map< string, unordered_map<string, float> > angles_df;
    unordered_map< string, unordered_map<string, int> > n_routes_df;

    float number_of_missing_zones_data = 0.0;


    for(int i = 0; i < nb_nodes+1; i++){
        token = *next(ptr_3, 0);
        historic_zones_id[i] = token;
        advance(ptr_3, 1);
    }

    //READ PROBABILITY MATRIX FILE
    token = *next(ptr, 1);
    float fill_prob_nan = 0.5;
    string fill_prob_nan_str = "0.5";

    //UNCOMMENT THIS SECTION WHEN PROB_MATRIX HAS A WORD/ELEMENT IN ROW 0 COLUMN 0, OTHERWISE, LEAVE COMMENT
    advance(ptr, 1);
    token = *next(ptr, 1);

    for(int i = 0; i < nb_nodes; i++){
        if (i == 0){
            for(int k = 0; k < nb_nodes-1; k++){
                token = *next(ptr, 1);
                advance(ptr, 1);
            }
        }
        token = *next(ptr, 1);
        advance(ptr, 1);
        zone_1 = historic_zones_id[i];

        for(int j = 0; j < nb_nodes; j++){
            zone_2 = historic_zones_id[j];
            token = *next(ptr, 1);
            if (token == "nan"){
                token = fill_prob_nan_str;
            }
            if (token == "0.5")
            {
                number_of_missing_zones_data++;
            }
            probabilities_matrix[i][j] = stof(token);
            probabilities_df[zone_1][zone_2] = stof(token);
            advance(ptr, 1);
        }
        probabilities_matrix[i][nb_nodes] = fill_prob_nan;
        probabilities_df[zone_1][historic_zones_id[nb_nodes]] = fill_prob_nan;
        zone_3 = historic_zones_id[nb_nodes];
        probabilities_df[zone_3][zone_1] = fill_prob_nan;
    }
    zone_1 = historic_zones_id[nb_nodes];
    probabilities_df[zone_1][zone_1] = fill_prob_nan;

    float percentage_missing_data_matrix = number_of_missing_zones_data/(nb_nodes *nb_nodes);
    cout << "Parsed HEADING MATRIX correctly" << endl;
    //READ ANGLES MATRIX FILE
    token = *next(ptr_4, 1);
    //advance(ptr, 1);

    advance(ptr_4, 1);
    token = *next(ptr_4, 1);

    for(int i = 0; i < nb_nodes; i++){
        if (i == 0){
            for(int k = 0; k < nb_nodes-1; k++){
                token = *next(ptr_4, 1);
                advance(ptr_4, 1);
            }
        }
        token = *next(ptr_4, 1);
        advance(ptr_4, 1);
        zone_1 = historic_zones_id[i];

        for(int j = 0; j < nb_nodes; j++){
            zone_2 = historic_zones_id[j];
            token = *next(ptr_4, 1);
            if (token == "nan"){
                token = fill_prob_nan_str;
            }
            angles_matrix[i][j] = stof(token);
            angles_df[zone_1][zone_2] = stof(token);
            advance(ptr_4, 1);
        }
        angles_matrix[i][nb_nodes] = fill_prob_nan;
        angles_df[zone_1][historic_zones_id[nb_nodes]] = fill_prob_nan;
        zone_3 = historic_zones_id[nb_nodes];
        angles_df[zone_3][zone_1] = fill_prob_nan;
    }
    zone_1 = historic_zones_id[nb_nodes];
    angles_df[zone_1][zone_1] = fill_prob_nan;
    cout << "Parsed ANGLE MATRIX correctly" << endl;

    cout << " Done reading historic files" << endl;

    return {probabilities_matrix, n_routes_matrix, historic_zones_id, probabilities_df, angles_df, n_routes_df, max_n_routes};
}

