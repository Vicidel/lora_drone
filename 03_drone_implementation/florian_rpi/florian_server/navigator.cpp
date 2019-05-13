#include "navigator.h"

using namespace std;

void* nav(void* arg){
	static int waiting_cnt = 0;
	string dummy;
	const string one = "1";
	sleep(1);

	while(1) {
		mav &mavlink = *(static_cast<mav*>(arg));

		if(mavlink.cloud_ctrl == -1){
			cout << "\nPress 1 for PID on Cloud" << endl;
			cin >> dummy;
			if(dummy.compare(one)){
				mavlink.cloud_ctrl = 0;
			} else {
				mavlink.cloud_ctrl = 1;
			}
		}

		// while loop until connexion to the rpi
		if(!mavlink.close_conn){
			cout << "\nConnexion established.\n";
			sleep(1);
			do {
				switch(waiting_cnt){
					case 0:
						cout << "\rWaiting for GPS lock (.  )" << flush;
						waiting_cnt++;
						break;
					case 1:
						cout << "\rWaiting for GPS lock (.. )" << flush;
						waiting_cnt++;
						break;
					case 2:
						cout << "\rWaiting for GPS lock (...)" << flush;
						waiting_cnt = 0;
						break;
				}
				sleep(1);
				//printf("%f",mavlink.local_position_ned.x);
			} while(!mavlink.close_conn && fabs(mavlink.local_position_ned.x) < GPS_CHECK);
		}
		else{
			cout << flush;
			switch(waiting_cnt){
				case 0:
					cout << "\rWaiting for connexion (.  )" << flush;
					waiting_cnt++;
					break;
				case 1:
					cout << "\rWaiting for connexion (.. )" << flush;
					waiting_cnt++;
					break;
				case 2:
					cout << "\rWaiting for connexion (...)" << flush;
					waiting_cnt = 0;
					break;
			}
		}

		sleep(1);

		// entering offboard control user interface
		// to add setpoint in local ned position
		while(!mavlink.close_conn && fabs(mavlink.local_position_ned.x) > GPS_CHECK) {
			
			set_position(mavlink);

			if(mavlink.close_conn){
				cout << "/!\\ Connexion lost /!\\" << endl;
				break;
			}
			if(fabs(mavlink.local_position_ned.x) < GPS_CHECK){
				cout << "/!\\ GPS lock lost /!\\" << endl;
				break;
			}
		}
	}
}

// Api to set waypoint from the terminal
int set_position(mav &mavlink){
	//printf("ctrl mode : %d\n", mavlink.cloud_ctrl);
	string dummy;
	const string one = "1";
	const string p = "p";
	string new_setpoint[3];
	string::size_type sz;

	cout << "\nCurrent Position: <" << mavlink.local_position_ned.x << ";"
									<< mavlink.local_position_ned.y << ";"
									<< mavlink.local_position_ned.z << " >"
									<< endl;
	cout << flush;
	cout << "Enter x y z coordonates: ";



	cin >> new_setpoint[0];
	// print current position on terminal
	if(!new_setpoint[0].compare(p)){
		return 1;
	}
	cin >> new_setpoint[1];
	// print current position on terminal
	if(!new_setpoint[1].compare(p)){
		return 1;
	}
	cin >> new_setpoint[2];
	// print current position on terminal
	if(!new_setpoint[2].compare(p)){
		return 1;
	}
	cout << flush;

	cout << "New waypoint set: < " << new_setpoint[0] << ";"
								   << new_setpoint[1] << ";"
								   << new_setpoint[2] << " >"
								   << endl;
	cout << "Press 1 to confirm ";
	cin >> dummy;
	if (!dummy.compare(one)){
		for(int i=0;i<3;i++){
			mavlink.setpoint[i] = stof(new_setpoint[i],&sz);
			//printf("%f\n", mavlink.setpoint[i]);
		}
		cout << "Confirmed. Flying to new waypoint." << endl;
	} else {
		cout << "Cancelled." << endl;
	}
	sleep(2);
	return 1;
}