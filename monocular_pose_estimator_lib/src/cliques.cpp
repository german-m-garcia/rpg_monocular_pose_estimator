/*
 * cliques.cpp
 *
 *  Created on: Oct 19, 2017
 *      Author: tomatito
 *  Code from    http://www.sanfoundry.com/cpp-program-find-all-cliques-given-size-k/
 */

    #include <iostream>
    #include <fstream>
    #include <string>
    #include <vector>
	#include <monocular_pose_estimator_lib/cliques.h>
    using namespace std;


    bool removable(vector<int> neighbor, vector<int> cover);

    int max_removable(vector<vector<int> > neighbors, vector<int> cover);

    vector<int> procedure_1(vector<vector<int> > neighbors, vector<int> cover);

    vector<int> procedure_2(vector<vector<int> > neighbors, vector<int> cover,

            int k);

    int cover_size(vector<int> cover);

    ifstream infile("graph.txt");

    ofstream outfile("cliques.txt");




    //given an adjacency matrix encoded in graph, produces the neighbors matrix
    void build_graph(vector<vector<int> >& graph, vector<vector<int> >& neighbors){
    	neighbors.resize(0);
    	for (int i = 0; i < graph.size(); i++)  {
			vector<int> neighbor;
			for (int j = 0; j < graph[i].size(); j++)
				if (graph[i][j] == 1)
					neighbor.push_back(j);
			neighbors.push_back(neighbor);
		}

    }

    bool cliques_equal(const vector<int> clique_1, const vector<int> clique_2){

    	for(int i=0;i<clique_1.size();i++){
    		if (clique_1[i] != clique_2[i])
    			return false;
    	}
    	return true;


    }

    bool clique_exists(const vector<vector<int> >& cliques, const vector<int> clique){

    	for(int i=0;i<cliques.size();i++){

    		if(cliques_equal(cliques[i], clique))
    			return true;

    	}
    	return false;
    }

    void invert_graph(vector<vector<int> >& graph_in,vector<vector<int> >& graph_out){

		int n = graph_in.size();
    	for (int i = 0; i < n; i++)  {

			vector<int> row;
			for (int j = 0; j < n; j++) {
				if (graph_in[i][j] == 0)
					row.push_back(1);
				else
					row.push_back(0);
			}
			graph_out.push_back(row);
		}
    }


    /**
     * graph: input graph adjacency matrix with 1s meaning edge
     * K: clique size to look for
     * cliques: the output cliques that were found
     *
     */
    void process_cliques(vector<vector<int> >& graph, int K ,  vector<vector<int> >& cliques){
    	//number of vertices
    	int n = graph.size();
    	int k = n - K;
    	int counter, s = 0;
    	vector<vector<int> > neighbors;
    	build_graph(graph, neighbors);
    	//Find Cliques
		bool found = false;

		cout << "Finding Cliques..." << endl;
		int min = n + 1;
		vector<vector<int> > covers;
		vector<int> allcover;

		for (int i = 0; i < graph.size(); i++)
			allcover.push_back(1);

		for (int i = 0; i < allcover.size(); i++){
			//if (found)
				//break;

			counter++;
			vector<int> cover = allcover;
			cover[i] = 0;
			cover = procedure_1(neighbors, cover);
			s = cover_size(cover);

			if (s < min)
				min = s;

			if (s <= k) {
				vector<int> clique;
				for (int j = 0; j < cover.size(); j++)
					if (cover[j] == 0)
						clique.push_back(j+1);
						//outfile << j + 1 << " ";


				if (!clique_exists(cliques, clique))
					cliques.push_back(clique);
				covers.push_back(cover);
				found = true;
				//break;
			}

			for (int j = 0; j < n - k; j++)
				cover = procedure_2(neighbors, cover, j);

			s = cover_size(cover);
			if (s < min)
				min = s;

			vector<int> clique;
			for (int j = 0; j < cover.size(); j++)
				if (cover[j] == 0)
					clique.push_back(j+1);
					//outfile << j + 1 << " ";

			if (!clique_exists(cliques, clique))
				cliques.push_back(clique);
			covers.push_back(cover);

			if (s <= k) {
				found = true;
				//break;
			}
		}

		//Pairwise Intersections
		for (int p = 0; p < covers.size(); p++){

			if (found)
			   break;

			for (int q = p + 1; q < covers.size(); q++){

				if (found)
					break;

				counter++;

				vector<int> cover = allcover;
				for (int r = 0; r < cover.size(); r++)
					if (covers[p][r] == 0 && covers[q][r] == 0)
						cover[r] = 0;
				cover = procedure_1(neighbors, cover);
				s = cover_size(cover);
				if (s < min)
					min = s;

				if (s <= k) {
					vector<int> clique;
					for (int j = 0; j < cover.size(); j++)
						if (cover[j] == 0)
							clique.push_back(j+1);
							//outfile << j + 1 << " ";

					if (!clique_exists(cliques, clique))
						cliques.push_back(clique);
					found = true;
					break;
				}

				for (int j = 0; j < k; j++)
					cover = procedure_2(neighbors, cover, j);

				s = cover_size(cover);
				if (s < min)
					min = s;

				vector<int> clique;
				for (int j = 0; j < cover.size(); j++)
					if (cover[j] == 0)
						clique.push_back(j+1);
						//outfile << j + 1 << " ";

				if (!clique_exists(cliques, clique))
					cliques.push_back(clique);
				if (s <= k){
					found = true;
					break;
				}
			}
		}


	}

    void debug_graph(vector<vector<int> >& graph){
    	for(int i=0;i<graph.size();i++){
    		for(int j=0;j<graph[i].size();j++){
    			std::cout << graph[i][j]<<" ";
    		}
    		std::cout <<std::endl;
    	}
    }

    void find_cliques(vector<vector<int> >& graph, int K ,  vector<vector<int> >& cliques){
    	vector<vector<int> > graph_inverted;
    	invert_graph( graph,graph_inverted);
    	debug_graph(graph_inverted);
    	process_cliques(graph_inverted,  K ,   cliques);
    }



    int main(){
    	 //Read Graph (note we work with the complement of the input graph)

		cout << "Clique Algorithm." << endl;
		int n, i, j, k, K, p, q, r, s, min, edge, counter = 0;
		//read number of vertices
		infile >> n;

		//builds the graph
		vector<vector<int> > graph;
		for (i = 0; i < n; i++)  {

			vector<int> row;
			for (j = 0; j < n; j++) {
				infile >> edge;
				if (edge == 0)
					row.push_back(1);
				else
					row.push_back(0);
			}
			graph.push_back(row);
		}
		cout << "Graph has n = " << n << " vertices." << endl;

		//Read maximum size of Clique wanted

		cout << "Find a Clique of size at least k = ";
		cin >> K;
		vector<vector<int> > cliques;
		process_cliques( graph,  K ,  cliques);
		//print the cliques
		debug_graph(cliques);

    }

    int main_(){

        //Read Graph (note we work with the complement of the input graph)

        cout << "Clique Algorithm." << endl;
        int n, i, j, k, K, p, q, r, s, min, edge, counter = 0;
        //read number of vertices
        infile >> n;

        //builds the graph
        vector<vector<int> > graph;
        for (i = 0; i < n; i++)  {

            vector<int> row;
            for (j = 0; j < n; j++) {
                infile >> edge;
                if (edge == 0)
                    row.push_back(1);
                else
                    row.push_back(0);
            }
            graph.push_back(row);
        }


        //Find Neighbors
        vector<vector<int> > neighbors;
        for (i = 0; i < graph.size(); i++)  {
            vector<int> neighbor;
            for (j = 0; j < graph[i].size(); j++)
                if (graph[i][j] == 1)
                    neighbor.push_back(j);
            neighbors.push_back(neighbor);
        }

        cout << "Graph has n = " << n << " vertices." << endl;

        //Read maximum size of Clique wanted

        cout << "Find a Clique of size at least k = ";
        cin >> K;
        k = n - K;

        //Find Cliques
        bool found = false;

        cout << "Finding Cliques..." << endl;
        min = n + 1;
        vector<vector<int> > covers;
        vector<int> allcover;

        for (i = 0; i < graph.size(); i++)
            allcover.push_back(1);

        for (i = 0; i < allcover.size(); i++)
        {

            //if (found)
                //break;

            counter++;
            cout << counter << ". ";
            outfile << counter << ". ";
            vector<int> cover = allcover;
            cover[i] = 0;
            cover = procedure_1(neighbors, cover);
            s = cover_size(cover);

            if (s < min)
                min = s;

            if (s <= k) {

                outfile << "First Clique (" << n - s << "): ";
                for (j = 0; j < cover.size(); j++)
                    if (cover[j] == 0)
                        outfile << j + 1 << " ";

                outfile << endl;

                cout << "Clique Size: " << n - s << endl;
                covers.push_back(cover);
                found = true;
                //break;
            }

            for (j = 0; j < n - k; j++)
                cover = procedure_2(neighbors, cover, j);

            s = cover_size(cover);
            if (s < min)
                min = s;
            outfile << "Second Clique (" << n - s << "): ";

            for (j = 0; j < cover.size(); j++)
                if (cover[j] == 0)
                    outfile << j + 1 << " ";
            outfile << endl;
            cout << "Clique Size: " << n - s << endl;
            covers.push_back(cover);

            if (s <= k) {

                found = true;

                //break;
            }
        }

        //Pairwise Intersections
        for (p = 0; p < covers.size(); p++){

            if (found)
               break;

            for (q = p + 1; q < covers.size(); q++){

                if (found)
                    break;

                counter++;
                cout << counter << ". ";
                outfile << counter << ". ";
                vector<int> cover = allcover;
                for (r = 0; r < cover.size(); r++)
                    if (covers[p][r] == 0 && covers[q][r] == 0)
                        cover[r] = 0;
                cover = procedure_1(neighbors, cover);
                s = cover_size(cover);
                if (s < min)
                    min = s;

                if (s <= k) {
                    outfile << "Clique (" << n - s << "): ";
                    for (j = 0; j < cover.size(); j++)
                        if (cover[j] == 0)
                            outfile << j + 1 << " ";

                    outfile << endl;
                    cout << "Clique Size: " << n - s << endl;
                    found = true;
                    break;
                }

                for (j = 0; j < k; j++)
                    cover = procedure_2(neighbors, cover, j);

                s = cover_size(cover);
                if (s < min)
                    min = s;

                outfile << "Clique (" << n - s << "): ";
                for (j = 0; j < cover.size(); j++)
                    if (cover[j] == 0)
                        outfile << j + 1 << " ";

                outfile << endl;
                cout << "Clique Size: " << n - s << endl;

                if (s <= k){
                    found = true;
                    break;
                }
            }
        }

        if (found)
            cout << "Found Clique of size at least " << K << "." << endl;

        else
            cout << "Could not find Clique of size at least " << K << "." << endl
                    << "Maximum Clique size found is " << n - min << "." << endl;
        cout << "See cliques.txt for results." << endl;
        return 0;

    }



    bool removable(vector<int> neighbor, vector<int> cover)

    {

        bool check = true;

        for (int i = 0; i < neighbor.size(); i++)
            if (cover[neighbor[i]] == 0) {
                check = false;
                break;
            }
        return check;

    }



    int max_removable(vector<vector<int> > neighbors, vector<int> cover)

    {

        int r = -1, max = -1;
        for (int i = 0; i < cover.size(); i++){

            if (cover[i] == 1 && removable(neighbors[i], cover) == true){

                vector<int> temp_cover = cover;
                temp_cover[i] = 0;
                int sum = 0;
                for (int j = 0; j < temp_cover.size(); j++)
                    if (temp_cover[j] == 1 && removable(neighbors[j], temp_cover) == true)
                        sum++;
                if (sum > max) {

                    max = sum;
                    r = i;
                }
            }
        }

        return r;

    }



    vector<int> procedure_1(vector<vector<int> > neighbors, vector<int> cover)

    {

        vector<int> temp_cover = cover;
        int r = 0;
        while (r != -1){
            r = max_removable(neighbors, temp_cover);
            if (r != -1)
                temp_cover[r] = 0;
        }

        return temp_cover;

    }



    vector<int> procedure_2(vector<vector<int> > neighbors, vector<int> cover,

            int k)

    {

        int count = 0;
        vector<int> temp_cover = cover;
        int i = 0;

        for (int i = 0; i < temp_cover.size(); i++){

            if (temp_cover[i] == 1){

                int sum = 0, index;
                for (int j = 0; j < neighbors[i].size(); j++)
                    if (temp_cover[neighbors[i][j]] == 0) {
                        index = j;
                        sum++;
                    }

                if (sum == 1 && cover[neighbors[i][index]] == 0){

                    temp_cover[neighbors[i][index]] = 1;
                    temp_cover[i] = 0;
                    temp_cover = procedure_1(neighbors, temp_cover);
                    count++;
                }

                if (count > k)
                    break;
            }
        }
        return temp_cover;

    }



    int cover_size(vector<int> cover){

        int count = 0;
        for (int i = 0; i < cover.size(); i++)
            if (cover[i] == 1)
                count++;
        return count;

    }


