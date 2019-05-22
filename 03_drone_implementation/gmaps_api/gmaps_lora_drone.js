//
//    File: gmaps_lora_drone.html
//    Author: Victor Delafontaine
//    Date: May 2019
//    
//    Google Maps API code to create the GUI for the drone.
//    Receives information from the Swisscom server app.
//




//#########################################################################################
//#################################  GLOBAL VARIABLES #####################################
//#########################################################################################

// Firebase reference
var firebase = new Firebase('https://drone-3bd2a.firebaseio.com/');

// create base coordinates
var lat_zurich = 47.39784;
var lng_zurich = 8.54558;
var lat_lausanne = 46.5165372;
var lng_lausanne = 6.5628348;

// storage            
var lat_homeR, lng_homeR, lat_homeG, lng_homeG, lat_homeB, lng_homeB;
var lat_droneR, lng_droneR, lat_droneG, lng_droneG, lat_droneB, lng_droneB;

// drones states
var stateR = "UNKNOWN";
var stateG = "UNKNOWN";
var stateB = "UNKNOWN";

// drone altitudes (relative)
var altitudeR = 0;
var altitudeG = 0;
var altitudeB = 0;

// geofence
var geofence_radius = 0;        // TODO: 0 because can't click through circle...

// listener active for click
var click_listener_active = false

// after click on RGB takeoff, go in none mode after X milliseconds 
var delay_after_takeoff = 8000;

// checks drone onlines ready for takeoff and server online
var delay_online_check = 500;





//#########################################################################################
//##################################  MAIN FUNCTION #######################################
//#########################################################################################

// main function called from the html file
function main() {

    // define the map itself
    var map = new google.maps.Map(document.getElementById('map'), {
        center: {lat: lat_zurich, lng: lng_zurich},
        zoom: 16,
        styles: [{
            featureType: 'poi',
            stylers: [{ visibility: 'off' }]  // Turn off POI.
        },
        {
            featureType: 'transit.station',
            stylers: [{ visibility: 'off' }]  // Turn off bus, train stations...
        }],
        disableDoubleClickZoom: false,
        streetViewControl: false,
    });

    // centering controls
    init_centering_controls(map);

    // things on the map
    var markers = create_markers(map);      // create marker
    var wp_lists = create_waypoint_lists(); // create waypoint lists
    var circles = create_circles();         // create circles
    var paths = create_paths(map);          // create paths

    // listener for clicks
    map.addListener('click', function(e) {
        // check is listener is active
        if(click_listener_active){
            // get coordinates 
            var lat = e.latLng.lat();
            var lng = e.latLng.lng();

            // post network estimate on server
            const url='http://victor.scapp.io/lora/network_estimate_latlng';
            const data={'lat': lat, 'lng': lng}
            axios({method: 'POST', url: url, data: data})
        }
    });

    // legend
    init_legend(map);

    // start Firebase and database listeners
    init_firebase_homes(map, markers, circles);                  // homes
    init_firebase_drones(map, markers, paths);          // drones, paths
    init_firebase_waypoints(map, wp_lists);             // waypoints
    init_firebase_estimations(map, markers, circles);   // estimations

    // initial button colors: call callbacks as if clicked
    takeoff_button_cb(this, 'X');
    kill_button_cb(this, 0);
    network_button_cb(this, 'none');

    // initialize the periodic check of drone and server online
    check_drone_online();
    check_server_online();

    // fills the parameters and state fields
    get_base_param();
    change_states(stateR, stateG, stateB, altitudeR, altitudeG, altitudeB);
}




//#########################################################################################
//###############################  PARAMETERS AND STATE ###################################
//#########################################################################################

// function when parameter is changed
function param_change(){
    // get from HTML elements
    var loop_todo = document.getElementById("loop_todo").value;
    var rad1 = document.getElementById("rad1").value;
    var rad2 = document.getElementById("rad2").value;
    var hover = document.getElementById("hover").value;
    var flight = document.getElementById("flight").value;
    var takeoff = document.getElementById("takeoff").value;

    // POST on server
    const url='http://victor.scapp.io/param/change_from_maps';
    const data={'rad1': rad1, 'rad2': rad2, 'hover': hover, 'loop_todo': loop_todo, 'flight': flight, 'takeoff': takeoff};
    axios({method: 'POST', url: url, data: data})
    .then(function(response){
        window.alert("Success!")
    })
    .catch(function(error){
        window.alert("Fail...")
    })
}

// init the params with real server values
function get_base_param(){

    // get params from server
    const url='http://victor.scapp.io/param/print';
    axios({method: 'GET', url: url}).then(function(response){
        document.getElementById("loop_todo").value = response.data['loops to do'];
        document.getElementById("rad1").value = response.data['radius']['v1'];
        document.getElementById("rad2").value = response.data['radius']['v2'];
        document.getElementById("hover").value = response.data['hovering time'];
        document.getElementById("flight").value = response.data['altitude']['flying'];
        document.getElementById("takeoff").value = response.data['altitude']['takeoff'];
    });
}

// set the states in GUI
function change_states(stateR, stateG, stateB, altitudeR, altitudeG, altitudeB){
    // change the string in HTML stuff
    document.getElementById('legend').children[4].innerHTML = "<img src=marker/droneR.png width=20> Drone 1/R: "+stateR+", "+altitudeR;
    document.getElementById('legend').children[5].innerHTML = "<img src=marker/droneG.png width=20> Drone 2/G: "+stateG+", "+altitudeG;
    document.getElementById('legend').children[6].innerHTML = "<img src=marker/droneB.png width=20> Drone 3/B: "+stateB+", "+altitudeB;
}

// testing if drone is online
function check_drone_online(){

    // check online from server
    const url='http://victor.scapp.io/param/check_online';
    axios({method: 'POST', url: url}).then(function(response){
        
        // booleans
        droneR_online = response.data['droneR'];
        droneG_online = response.data['droneG'];
        droneB_online = response.data['droneB'];

        // R
        if(droneR_online==true)
            document.getElementById('R').disabled = false;
        else
            document.getElementById('R').disabled = true;

        // G
        if(droneG_online==true)
            document.getElementById('G').disabled = false;
        else
            document.getElementById('G').disabled = true;

        // B
        if(droneB_online==true)
            document.getElementById('B').disabled = false;
        else
            document.getElementById('B').disabled = true;

        // all
        if( (droneR_online==true) && (droneG_online==true) && (droneB_online==true))
            document.getElementById('all').disabled = false;
        else
            document.getElementById('all').disabled = true;

        // call function again after X seconds
        setTimeout(check_drone_online, delay_online_check);
    });
}

// testing if server is online
function check_server_online(){
    
    // check server
    const url='http://victor.scapp.io/';
    axios({method: 'HEAD', url: url})
    .catch(function (error){
        window.alert("Server is offline, OK to try again")
    })
    .finally(function () {
        // call function again after X seconds
        setTimeout(check_server_online, delay_online_check);
    });
}



//#########################################################################################
//################################  MAPS AND CONTROL ######################################
//#########################################################################################

// init centering controls
function init_centering_controls(map) {

    // create div for centering button
    var centerControlDiv = document.createElement('div');
    var centerControl = new center_control(centerControlDiv, map, lat_lausanne, lng_lausanne, "Lausanne");
    centerControlDiv.index = 1;
    map.controls[google.maps.ControlPosition.TOP_CENTER].push(centerControlDiv);

    // create div for centering button
    var centerControlDiv2 = document.createElement('div');
    var centerControl2 = new center_control(centerControlDiv2, map, lat_zurich, lng_zurich, "Zurich");
    centerControlDiv2.index = 1;
    map.controls[google.maps.ControlPosition.TOP_CENTER].push(centerControlDiv2);

    // create div for centering button
    var centerControlDiv3 = document.createElement('div');
    var centerControl3 = new center_control(centerControlDiv3, map, 0, 0, "Zero");
    centerControlDiv3.index = 1;
    map.controls[google.maps.ControlPosition.TOP_CENTER].push(centerControlDiv3);
}

// creates a GUI for centering map
function center_control(control_div, map, lat, lng, string){

    // Set CSS for the control border.
    var controlUI = document.createElement('div');
    controlUI.style.backgroundColor = '#fff';
    controlUI.style.border = '2px solid #fff';
    controlUI.style.borderRadius = '3px';
    controlUI.style.boxShadow = '0 2px 6px rgba(0,0,0,.3)';
    controlUI.style.cursor = 'pointer';
    controlUI.style.marginBottom = '22px';
    controlUI.style.textAlign = 'center';
    controlUI.title = 'Click to recenter the map';
    control_div.appendChild(controlUI);

    // Set CSS for the control interior.
    var controlText = document.createElement('div');
    controlText.style.color = 'rgb(25,25,25)';
    controlText.style.fontFamily = 'Roboto,Arial,sans-serif';
    controlText.style.fontSize = '14px';
    controlText.style.lineHeight = '20px';
    controlText.style.paddingLeft = '5px';
    controlText.style.paddingRight = '5px';
    controlText.innerHTML = string;
    controlUI.appendChild(controlText);

    // Setup the click event listeners: simply set the map to Chicago.
    controlUI.addEventListener('click', function() {
        console.log("Centering on "+string)
        map.setCenter(new google.maps.LatLng(lat, lng));
        map.setZoom(16);
    });
}

// init legend
function init_legend(map){
    // create legend object
    var legend = document.getElementById('legend');

    // add homeR
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/homeR.png width=20> Home 1/R'
    legend.appendChild(div);

    // add homeG
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/homeG.png width=20> Home 2/G'
    legend.appendChild(div);

    // add homeB
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/homeB.png width=20> Home 3/B'
    legend.appendChild(div);

    // add droneR
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/droneR.png width=20> Drone 1/R'
    legend.appendChild(div);

    // add droneG
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/droneG.png width=20> Drone 2/G'
    legend.appendChild(div);

    // add droneB
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/droneB.png width=20> Drone 3/B'
    legend.appendChild(div);

    // add waypoint past
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/waypointR_old_circle.png width=20> Drone 1/R past waypoint'
    legend.appendChild(div);

    // add waypoint current
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/waypointR_current_circle.png width=20> Drone 1/R current waypoint'
    legend.appendChild(div);

    // add waypoint past
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/waypointG_old_circle.png width=20> Drone 2/G past waypoint'
    legend.appendChild(div);

    // add waypoint current
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/waypointG_current_circle.png width=20> Drone 2/G current waypoint'
    legend.appendChild(div);

    // add waypoint past
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/waypointB_old_circle.png width=20> Drone 3/B past waypoint'
    legend.appendChild(div);

    // add waypoint current
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/waypointB_current_circle.png width=20> Drone 3/B current waypoint'
    legend.appendChild(div);

    // add network
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/network_circle.png width=20> Network estimate'
    legend.appendChild(div);

    // add est1
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/estimate2_circle.png width=20> First estimation'
    legend.appendChild(div);

    // add est2
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/estimate_circle.png width=20> Second estimation'
    legend.appendChild(div);

    // push on map
    map.controls[google.maps.ControlPosition.RIGHT_BOTTOM].push(legend);
}

// get lora network estimate
function get_lora_network_estimate() {

    // get network est from server
    const url='http://victor.scapp.io/lora/compute_network_est';
    axios({method: 'GET', url: url}).then(function(response){
        if(response.data=='No location received...') {
            console.log(response.data)
            window.alert("No location data was ever received by the server, please place network manually");
            network_button_cb(this, 'none');
        }
        else{
            console.log(response.data)
            window.alert(response.data)
            //var lat = parseFloat(response.data.substring(4+response.data.indexOf('lat='), response.data.length))
            //var lng = parseFloat(response.data.substring(4+response.data.indexOf('lng='), response.data.length))
            network_button_cb(this, 'none');
        }
    })
}




//#########################################################################################
//################################  FIREBASE OBJECTS ######################################
//#########################################################################################

// init Firebase for home
function init_firebase_homes(map, markers, circles) {

    // reference in Firebase
    var home_refR = firebase.child('homeR');
    var home_refG = firebase.child('homeG');
    var home_refB = firebase.child('homeB');

    // for homeR
    home_refR.on('child_added', function(snapshot){

        // get coordinates
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // move marker
        markers.homeR.setPosition(new google.maps.LatLng(lat, lng));
        markers.homeR.setMap(map);

        // move geofence circle
        circles.geoR.setCenter(new google.maps.LatLng(lat, lng));
        circles.geoR.setRadius(geofence_radius);
        circles.geoR.setMap(map);

        // set start as home
        lat_homeR = lat;
        lng_homeR = lng;
    });
    home_refR.on('child_removed', function(snapshot) {
        markers.homeR.setMap(null)
        circles.geoR.setMap(null)
    });

    // for homeG
    home_refG.on('child_added', function(snapshot){

        // get coordinates
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // move marker
        markers.homeG.setPosition(new google.maps.LatLng(lat, lng));
        markers.homeG.setMap(map);

        // move geofence circle
        circles.geoG.setCenter(new google.maps.LatLng(lat, lng));
        circles.geoG.setRadius(geofence_radius);
        circles.geoG.setMap(map);

        // set start as home
        lat_homeG = lat;
        lng_homeG = lng;
    });
    home_refG.on('child_removed', function(snapshot) {
        markers.homeG.setMap(null)
        circles.geoG.setMap(null)
    });

    // for homeB
    home_refB.on('child_added', function(snapshot){

        // get coordinates
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // move marker
        markers.homeB.setPosition(new google.maps.LatLng(lat, lng));
        markers.homeB.setMap(map);

        // move geofence circle
        circles.geoB.setCenter(new google.maps.LatLng(lat, lng));
        circles.geoB.setRadius(geofence_radius);
        circles.geoB.setMap(map);

        // set start as home
        lng_homeB = lat;
        lng_homeB = lng;
    });
    home_refB.on('child_removed', function(snapshot) {
        markers.homeB.setMap(null)
        circles.geoB.setMap(null)
    });
}

// init Firebase for drones and paths
function init_firebase_drones(map, markers, paths) {

    // reference in Firebase
    var drone_refR = firebase.child('droneR');
    var drone_refG = firebase.child('droneG');
    var drone_refB = firebase.child('droneB');

    // for droneR
    drone_refR.on('child_added', function(snapshot) {

        // get coordinates from firebase
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // move marker
        markers.droneR.setPosition(new google.maps.LatLng(lat, lng));
        markers.droneR.setMap(map);

        // draw path
        paths.listR.push({lat: lat, lng: lng});
        paths.lineR.setPath(paths.listR);
        paths.lineR.setMap(map)

        // change state in HTML
        stateR = snapshot.val().state;
        altitudeR = parseInt(snapshot.val().rel_alt);
        change_states(stateR, stateG, stateB, altitudeR, altitudeG, altitudeB);

        // change variable
        lat_droneR = lat;
        lng_droneR = lng;
    });
    drone_refR.on('child_removed', function(snapshot) {
        markers.droneR.setMap(null)
        paths.lineR.setMap(null)
        paths.listR = []
        change_states('UNKNOWN', 'UNKNOWN', 'UNKNOWN', 0, 0, 0);
    });

    // for droneG
    drone_refG.on('child_added', function(snapshot) {

        // get coordinates from firebase
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // move marker
        markers.droneG.setPosition(new google.maps.LatLng(lat, lng));
        markers.droneG.setMap(map);

        // draw path
        paths.listG.push({lat: lat, lng: lng});
        paths.lineG.setPath(paths.listG);
        paths.lineG.setMap(map)

        // change state in HTML
        stateG = snapshot.val().state;
        altitudeG = parseInt(snapshot.val().rel_alt);
        change_states(stateR, stateG, stateB, altitudeR, altitudeG, altitudeB);

        // change variable
        lat_droneG = lat;
        lng_droneG = lng;
    });
    drone_refG.on('child_removed', function(snapshot) {
        markers.droneG.setMap(null)
        paths.lineG.setMap(null)
        paths.listG = []
        change_states('UNKNOWN', 'UNKNOWN', 'UNKNOWN', 0, 0, 0);
    });

    // for droneB
    drone_refB.on('child_added', function(snapshot) {

        // get coordinates from firebase
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // move marker
        markers.droneB.setPosition(new google.maps.LatLng(lat, lng));
        markers.droneB.setMap(map);

        // draw path
        paths.listB.push({lat: lat, lng: lng});
        paths.lineB.setPath(paths.listB);
        paths.lineB.setMap(map)

        // change state in HTML
        stateB = snapshot.val().state;
        altitudeB = parseInt(snapshot.val().rel_alt);
        change_states(stateR, stateG, stateB, altitudeR, altitudeG, altitudeB);

        // change variable
        lat_droneB = lat;
        lng_droneB = lng;
    });
    drone_refB.on('child_removed', function(snapshot) {
        markers.droneB.setMap(null)
        paths.lineB.setMap(null)
        paths.listB = []
        change_states('UNKNOWN', 'UNKNOWN', 'UNKNOWN', 0, 0, 0);
    });
}

// init Firebase for waypoints
function init_firebase_waypoints(map, wp_lists) {

    // reference in Firebase
    var wayp_refR  = firebase.child('waypointR');
    var wayp_refG  = firebase.child('waypointG');
    var wayp_refB  = firebase.child('waypointB');

    // for waypoint R
    wayp_refR.on('child_added', function(snapshot) {
        // get coordinates from firebase
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // change color of last waypoints
        wp_lists.wpR.forEach(function(item, index, array){
            item.setIcon('marker/waypointR_old.png');
        });

        // create new marker
        var waypoint_marker = new google.maps.Marker({
            position: {lat: lat, lng: lng},
            map: map,
            icon: 'marker/waypointR_current.png',
            title: 'Waypoint',
        });

        // add it to list
        wp_lists.wpR.push(waypoint_marker);
    });
    wayp_refR.on('child_removed', function(snapshot) {
        wp_lists.wpR.forEach(function(item, index, array){
            item.setMap(null)
        });  
    });

    // for waypoint G
    wayp_refG.on('child_added', function(snapshot) {
        // get coordinates from firebase
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // change color of last waypoints
        wp_lists.wpG.forEach(function(item, index, array){
            item.setIcon('marker/waypointG_old.png');
        });

        // create new marker
        var waypoint_marker = new google.maps.Marker({
            position: {lat: lat, lng: lng},
            map: map,
            icon: 'marker/waypointG_current.png',
            title: 'Waypoint',
        });

        // add it to list
        wp_lists.wpG.push(waypoint_marker);
    });
    wayp_refG.on('child_removed', function(snapshot) {
        wp_lists.wpG.forEach(function(item, index, array){
            item.setMap(null)
        });  
    });

    // for waypoint B
    wayp_refB.on('child_added', function(snapshot) {
        // get coordinates from firebase
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // change color of last waypoints
        wp_lists.wpB.forEach(function(item, index, array){
            item.setIcon('marker/waypointB_current.png');
        });

        // create new marker
        var waypoint_marker = new google.maps.Marker({
            position: {lat: lat, lng: lng},
            map: map,
            icon: 'marker/waypointB_old.png',
            title: 'Waypoint',
        });

        // add it to list
        wp_lists.wpB.push(waypoint_marker);
    });
    wayp_refB.on('child_removed', function(snapshot) {
        wp_lists.wpB.forEach(function(item, index, array){
            item.setMap(null)
        });  
    });
}

// init Firebase for estimations
function init_firebase_estimations(map, markers, circles) {

    // reference in Firebase
    var netw_ref   = firebase.child('network');
    var est_ref    = firebase.child('estimate');

    // for network points
    netw_ref.on('child_added', function(snapshot) {

        // get coordinates from firebase
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // move marker
        markers.network.setPosition(new google.maps.LatLng(lat, lng));
        markers.network.setMap(map);
    });
    netw_ref.on('child_removed', function(snapshot) {
        //markers.network.setMap(null)
    });

    // for new estimation
    est_ref.on('child_added', function(snapshot) {
        // get coordinates from firebase
        var lat  = snapshot.val().lat;
        var lng  = snapshot.val().lng;
        var rad  = snapshot.val().radius;
        var type = snapshot.val().type;

        // different colors based on radius
        if(type=='network') {
            circles.c1.setCenter(new google.maps.LatLng(lat, lng));
            circles.c1.setMap(map);
            circles.c1.setRadius(rad);
            markers.network.setPosition(new google.maps.LatLng(lat, lng));
            markers.network.setMap(map);
        }
        if(type=='est1') {
            circles.c2.setCenter(new google.maps.LatLng(lat, lng));
            circles.c2.setMap(map);
            circles.c2.setRadius(rad);
            markers.est1.setPosition(new google.maps.LatLng(lat, lng));
            markers.est1.setMap(map);
        }
        if(type=='est2') {
            circles.c3.setCenter(new google.maps.LatLng(lat, lng));
            circles.c3.setMap(map);
            circles.c3.setRadius(rad);
            markers.est2.setPosition(new google.maps.LatLng(lat, lng));
            markers.est2.setMap(map);
        }
    });
    est_ref.on('child_removed', function(snapshot) {
        circles.c1.setMap(null);
        circles.c2.setMap(null);
        circles.c3.setMap(null);
        markers.est1.setMap(null);
        markers.est2.setMap(null);
    });
}   




//#########################################################################################
//################################  BUTTONS CALLBACKS #####################################
//#########################################################################################

// callback for killswitch button
function kill_button_cb(obj, kill){
    if(kill==1){
        console.log("Killing all drones now");
        document.getElementById('unkill').style.backgroundColor='#fff';
        document.getElementById('kill').style.backgroundColor='#aaa';
    }
    else{
        console.log("Unkilling all drones now");
        document.getElementById('unkill').style.backgroundColor='#aaa';
        document.getElementById('kill').style.backgroundColor='#fff';
    }

    // POST on server
    const url='http://victor.scapp.io/param/drone_kill';
    const data={'kill': kill}
    axios({method: 'POST', url: url, data: data})
}

// callback for takeoff button
function takeoff_button_cb(obj, drone_id) {
    //console.log("Starting drone: "+drone_id)

    // create data according to which button was pressed
    var data;
    if(drone_id=='R') {
        // data to send 
        data={'bool_drone1_start': 1, 'bool_drone2_start': 0, 'bool_drone3_start': 0};

        // buttons color
        document.getElementById('R').style.backgroundColor='#aaa';
        document.getElementById('G').style.backgroundColor='#fff';
        document.getElementById('B').style.backgroundColor='#fff';
        document.getElementById('X').style.backgroundColor='#fff';
        document.getElementById('all').style.backgroundColor='#fff';

        // go back in None after X seconds
        console.log("Setting drone "+drone_id+" to takeoff, disabling network estimate change");
        setTimeout(function(){console.log("Setting drone takeoff back to none ("+delay_after_takeoff/1000+" seconds passed)");takeoff_button_cb(this, 'X')}, delay_after_takeoff);

        // disable network estimate change
        network_button_cb(this, 'none');
    }
    if(drone_id=='G') {
        // data to send 
        data={'bool_drone1_start': 0, 'bool_drone2_start': 1, 'bool_drone3_start': 0};

        // buttons color
        document.getElementById('R').style.backgroundColor='#fff';
        document.getElementById('G').style.backgroundColor='#aaa';
        document.getElementById('B').style.backgroundColor='#fff';
        document.getElementById('X').style.backgroundColor='#fff';
        document.getElementById('all').style.backgroundColor='#fff';

        // go back in None after X seconds
        console.log("Setting drone "+drone_id+" to takeoff, disabling network estimate change");
        setTimeout(function(){console.log("Setting drone takeoff back to none ("+delay_after_takeoff/1000+" seconds passed)");takeoff_button_cb(this, 'X')}, delay_after_takeoff);

        // disable network estimate change
        network_button_cb(this, 'none');
    }
    if(drone_id=='B') {
        // data to send 
        data={'bool_drone1_start': 0, 'bool_drone2_start': 0, 'bool_drone3_start': 1};

        // buttons color
        document.getElementById('R').style.backgroundColor='#fff';
        document.getElementById('G').style.backgroundColor='#fff';
        document.getElementById('B').style.backgroundColor='#aaa';
        document.getElementById('X').style.backgroundColor='#fff';
        document.getElementById('all').style.backgroundColor='#fff';

        // go back in None after X seconds
        console.log("Setting drone "+drone_id+" to takeoff, disabling network estimate change");
        setTimeout(function(){console.log("Setting drone takeoff back to none ("+delay_after_takeoff/1000+" seconds passed)");takeoff_button_cb(this, 'X')}, delay_after_takeoff);

        // disable network estimate change
        network_button_cb(this, 'none');
    }
    if(drone_id=='X') {
        // data to send 
        data={'bool_drone1_start': 0, 'bool_drone2_start': 0, 'bool_drone3_start': 0};

        // buttons color
        document.getElementById('R').style.backgroundColor='#fff';
        document.getElementById('G').style.backgroundColor='#fff';
        document.getElementById('B').style.backgroundColor='#fff';
        document.getElementById('X').style.backgroundColor='#aaa';
        document.getElementById('all').style.backgroundColor='#fff';
    }
    if(drone_id=='all') {
        var confirmation = window.confirm("Are you sure?")

        if(confirmation==true){
            // data to send 
            data={'bool_drone1_start': 1, 'bool_drone2_start': 1, 'bool_drone3_start': 1};

            // buttons color
            document.getElementById('R').style.backgroundColor='#fff';
            document.getElementById('G').style.backgroundColor='#fff';
            document.getElementById('B').style.backgroundColor='#fff';
            document.getElementById('X').style.backgroundColor='#fff';
            document.getElementById('all').style.backgroundColor='#aaa';

            // go back in None after X seconds
            console.log("Setting all drones to takeoff, disabling network estimate change");
            setTimeout(function(){console.log("Setting drone takeoff back to none ("+delay_after_takeoff/1000+" seconds passed)");takeoff_button_cb(this, 'X')}, delay_after_takeoff);

            // disable network estimate change
            network_button_cb(this, 'none');
        }
        else{
            data={'bool_drone1_start': 0, 'bool_drone2_start': 0, 'bool_drone3_start': 0};
            takeoff_button_cb(this, 'X');
        }
    }

    // POST on server
    const url='http://victor.scapp.io/param/drone_start';
    axios({method: 'POST', url: url, data: data})
}

// callback for network estimate button
function network_button_cb(obj, type) {

    // 
    if(type=='get') {
        // button color
        document.getElementById('network_get').style.backgroundColor='#aaa';
        document.getElementById('network_place').style.backgroundColor='#fff';
        document.getElementById('network_none').style.backgroundColor='#fff';

        // click listener
        click_listener_active = false

        // call function
        get_lora_network_estimate();
    }
    if(type=='place') {
        // button color
        document.getElementById('network_get').style.backgroundColor='#fff';
        document.getElementById('network_place').style.backgroundColor='#aaa';
        document.getElementById('network_none').style.backgroundColor='#fff';

        // click listener
        click_listener_active = true
    }
    if(type=='none') {
        // button color
        document.getElementById('network_get').style.backgroundColor='#fff';
        document.getElementById('network_place').style.backgroundColor='#fff';
        document.getElementById('network_none').style.backgroundColor='#aaa';

        // click listener
        click_listener_active = false
    }
}




//#########################################################################################
//################################  ICONS AND MARKERS #####################################
//#########################################################################################

// creates the custom icons
function create_icons() {

    // drone icons
    var droneR_icon = {
        url: 'marker/droneR.png',
        anchor: new google.maps.Point(13, 13),
        scaledSize: new google.maps.Size(25, 25)
    };
    var droneG_icon = {
        url: 'marker/droneG.png',
        anchor: new google.maps.Point(13, 13),
        scaledSize: new google.maps.Size(25, 25)
    };
    var droneB_icon = {
        url: 'marker/droneB.png',
        anchor: new google.maps.Point(13, 13),
        scaledSize: new google.maps.Size(25, 25)
    };

    // home icon
    var homeR_icon = {
        url: 'marker/homeR.png',
        anchor: new google.maps.Point(13, 13),
        scaledSize: new google.maps.Size(25, 25)
    };
    var homeG_icon = {
        url: 'marker/homeG.png',
        anchor: new google.maps.Point(13, 13),
        scaledSize: new google.maps.Size(25, 25)
    };
    var homeB_icon = {
        url: 'marker/homeB.png',
        anchor: new google.maps.Point(13, 13),
        scaledSize: new google.maps.Size(25, 25)
    };

    // return them
    return {droneR: droneR_icon, droneG: droneG_icon, droneB: droneB_icon, homeR: homeR_icon, homeB: homeB_icon, homeG: homeG_icon};
}

// creates the markers
function create_markers(map, icons) {

    // create custom markers icons
    var icons = create_icons();

    // for drones
    var droneR_marker = new google.maps.Marker({
        map: map,
        icon: icons.droneR,
        title: 'Drone 1/R'
    });
    var droneG_marker = new google.maps.Marker({
        map: map,
        icon: icons.droneG,
        title: 'Drone 2/G'
    });
    var droneB_marker = new google.maps.Marker({
        map: map,
        icon: icons.droneB,
        title: 'Drone 3/B'
    });

    // for home
    var home_markerR = new google.maps.Marker({
        map: map,
        icon: icons.homeR,
        title: 'Home 1/R'
    });
    var home_markerG = new google.maps.Marker({
        map: map,
        icon: icons.homeG,
        title: 'Home 2/G'
    });
    var home_markerB = new google.maps.Marker({
        map: map,
        icon: icons.homeB,
        title: 'Home 3/B'
    });

    // for estimations
    var network_marker = new google.maps.Marker({
        map: map,
        icon: 'marker/network.png',
        title: 'Network estimate',
    });
    var est_marker1 = new google.maps.Marker({
        map: map,
        icon: 'marker/estimate2.png',
        title: 'First estimation',
    });
    var est_marker2 = new google.maps.Marker({
        map: map,
        icon: 'marker/estimate.png',
        title: 'Second estimation',
    });

    // return them 
    return {droneR: droneR_marker, droneG: droneG_marker, droneB: droneB_marker, homeR: home_markerR, homeG: home_markerG, homeB: home_markerB, network: network_marker, est1: est_marker1, est2: est_marker2};
}

// creates the circles
function create_circles() {

    // for network estimations
    var network_circle1 = new google.maps.Circle({
        strokeColor: '#FF0000',
        strokeOpacity: 0.5,
        strokeWeight: 2,
        fillColor: '#FF0000',
        fillOpacity: 0.1,
    })
    var network_circle2 = new google.maps.Circle({
        strokeColor: '#FFBB00',
        strokeOpacity: 0.5,
        strokeWeight: 2,
        fillColor: '#FF9900',
        fillOpacity: 0.2,
    })
    var network_circle3 = new google.maps.Circle({
        strokeColor: '#FFFF00',
        strokeOpacity: 0.5,
        strokeWeight: 2,
        fillColor: '#FFFF00',
        fillOpacity: 0.1,
    })

    // for geofences
    var home_circleR = new google.maps.Circle({
        strokeColor: '#550000',
        strokeOpacity: 1,
        strokeWeight: 1,
        fillColor: '#FFF',
        fillOpacity: 0,
    });
    var home_circleG = new google.maps.Circle({
        strokeColor: '#005500',
        strokeOpacity: 1,
        strokeWeight: 1,
        fillColor: '#FFF',
        fillOpacity: 0,
    });
    var home_circleB = new google.maps.Circle({
        strokeColor: '#000055',
        strokeOpacity: 1,
        strokeWeight: 1,
        fillColor: '#FFF',
        fillOpacity: 0,
    });

    return {c1: network_circle1, c2: network_circle2, c3: network_circle3, geoR: home_circleR, geoG: home_circleG, geoB: home_circleB};
}

// creates the waypoints lists (empty)
function create_waypoint_lists() {

    // return empty
    return {wpR: [], wpG: [], wpB: []};
    }

    // creates the drone paths lines and the list 
    function create_paths(map) { 

    // flight path
    var flight_pathR = new google.maps.Polyline({
        map: map,
        geodesic: true,
        strokeColor: '#DD0000',
        strokeOpacity: 1.0,
        strokeWeight: 3
    });
    var pathR = [];

    var flight_pathG = new google.maps.Polyline({
        map: map,
        geodesic: true,
        strokeColor: '#00CC00',
        strokeOpacity: 1.0,
        strokeWeight: 3
    });
    var pathG = [];

    var flight_pathB = new google.maps.Polyline({
        map: map,
        geodesic: true,
        strokeColor: '#0000DD',
        strokeOpacity: 1.0,
        strokeWeight: 3
    });
    var pathB = [];

    // return
    return {listR: [], lineR: flight_pathR, listG: [], lineG: flight_pathG, listB: [], lineB: flight_pathB};
}
