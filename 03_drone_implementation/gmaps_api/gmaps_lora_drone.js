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

// map, spider and info window as global
var map, oms, iw

// Firebase reference
var firebase = new Firebase('https://lora-drone-v2.firebaseio.com/');

// create base coordinates
var lat_zurich = 47.39784;
var lng_zurich = 8.54558;
var lat_lausanne = 46.5165372;
var lng_lausanne = 6.5628348;

// storage            
var lat_homeR, lng_homeR, lat_homeG, lng_homeG, lat_homeB, lng_homeB;
var lat_droneR, lng_droneR, lat_droneG, lng_droneG, lat_droneB, lng_droneB;
var lat_node, lng_node, lat_last_est, lng_last_est;

// drones states
var stateR = "UNKNOWN";
var stateG = "UNKNOWN";
var stateB = "UNKNOWN";

// drone altitudes (relative)
var altitudeR = 0;
var altitudeG = 0;
var altitudeB = 0;

// drone battery levels
var batteryR = 0;
var batteryG = 0;
var batteryB = 0;

// listener active for click
var click_listener_active = false

// after click on RGB takeoff, go in none mode after X milliseconds 
var delay_after_takeoff = 8000;

// checks drone onlines ready for takeoff and server online
var delay_online_check = 3000;

// to (de)activate buttons
var bool_some_drone_is_flying = false;

// colors for buttons
var color_can_click = '#fafafa';
var color_clicked = '#aaa';

// show the GUI objects or not
var bool_show_gui = true;





//#########################################################################################
//##################################  MAIN FUNCTION #######################################
//#########################################################################################

// main function called from the html file
function main() {

    // define the map itself
    map = new google.maps.Map(document.getElementById('map'), {
        center: {lat: lat_lausanne, lng: lng_lausanne},
        zoom: 16,
        scaleControl: true,
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

    // spider for overlapping markers: https://github.com/jawj/OverlappingMarkerSpiderfier
    oms = new OverlappingMarkerSpiderfier(map,{
        markersWontMove: true, 
        markersWontHide: true, 
        keepSpiderfied: true, 
        circleSpiralSwitchover: 200,
        nearbyDistance: 40,
        circleFootSeparation: 28,
        legWeight: 2
    });

    // information window
    iw = new google.maps.InfoWindow();

    // measuring tool
    measureTool = new MeasureTool(map, {
        showSegmentLength: true,
        tooltip: true,
        unit: MeasureTool.UnitTypeId.METRIC // metric, imperial, or nautical
    });

    // centering controls
    init_centering_controls();

    // things on the map
    var markers = create_markers();      // create marker
    var wp_lists = create_waypoint_lists(); // create waypoint lists
    var circles = create_circles();         // create circles
    var paths = create_paths();          // create paths

    // init listeners (keyboard and clicks)
    init_listeners();

    // set base values
    lat_node = 0
    lng_node = 0
    lat_last_est = 0
    lng_last_est = 0

    // show_gui checkbox
    init_show_gui();

    // legend, states, param, network buttons
    init_gui_elements();

    // start Firebase and database listeners
    firebase_homes(markers);                  // homes
    firebase_drones(markers, paths);          // drones, paths
    firebase_waypoints(wp_lists);             // waypoints
    firebase_estimations(markers, circles);   // estimations
    firebase_node(markers);                   // node ground truth

    // initial button colors: call callbacks as if clicked
    takeoff_button_cb('X');
    kill_button_cb(0);
    network_button_cb('none');

    // initialize the periodic check of drone and server online
    check_drone_online(markers);

    // fills the parameters and state fields
    get_base_param();
    change_states(stateR, stateG, stateB, altitudeR, altitudeG, altitudeB, batteryR, batteryG, batteryB);

    // print checklist
    print_checklist()

    // print results
    print_results()
}

// prints a checklist when loading the app
function print_checklist(){
    window.alert(`CHECKLIST:\n
    * check safety settings in QGC
        - geofence: action and radius
        - RTL: action and altitude
        - communication loss: action
    * change parameters in QGC
        - MPC_XY_VEL_MAX: maximum horizontal speed
        - MPC_TILTMAX_AIR: maximum tilt angle 
    * check radio calibration in QGC
    * if in a new testing place, run droneR code once
        - set the zero for conversions
        - otherwise problem in conversions
    * put all drones in MANUAL mode
        - drones will takeoff if in OFFBOARD
    * buttons are deactivated on drone takeoff 
        - if drone landed, press kill to activate again
    * safeties are always active
        - keyboard shortcuts khr
    * launch beaconing mode on the node
    * secret keys: >+"*ç
        - to print datasets
    * check that the gateways IDs are correct (secret key g)
    * TODO: add more
        `)
}

// init the keyboard listeners
function init_listeners(){

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

    // listener for keyboard
    document.onkeypress = function (e) {
        e = e || window.event;

        // empty Firebase
        if(e.keyCode==62){
            if(window.confirm("Key '>' pressed, do you want to empty the database?")){
                console.log("Deleting Firebase database")

                // empty database
                const url='http://victor.scapp.io/firebase/empty';
                const data={'id': 1}
                axios({method: 'POST', url: url, data: data})
                const data2={'id': 2}
                axios({method: 'POST', url: url, data: data2})
                const data3={'id': 3}
                axios({method: 'POST', url: url, data: data3})

                // clear states
                stateR = 'UNKNOWN'
                stateG = 'UNKNOWN'
                stateB = 'UNKNOWN'
                altitudeR = 0
                altitudeG = 0
                altitudeB = 0
                batteryR = 0
                batteryG = 0
                batteryB = 0
                change_states(stateR, stateG, stateB, altitudeR, altitudeG, altitudeB, batteryR, batteryG, batteryB);
            }
        }

        // print LoRa database
        if(e.keyCode==43){
            if(window.confirm("Key '+' pressed, do you want to popup-print the LoRa database?")){
                console.log("Popup-printing LoRa database")
                window.open('http://victor.scapp.io/lora/json');
            }
        }

        // print drone dataset
        if(e.keyCode==34){
            if(window.confirm(`Key '"' pressed, do you want to popup-print the drone dataset?`)){
                console.log("Popup-printing drone dataset")
                window.open('http://victor.scapp.io/print/drone_dataset');
            }
        }

        // print tri_dataset
        if(e.keyCode==42){
            if(window.confirm(`Key '*' pressed, do you want to popup-print the trilateration dataset?`)){
                console.log("Popup-printing trilateration dataset")
                window.open('http://victor.scapp.io/print/tri_dataset');
            }
        }

        // print tri_dataset_temp
        if(e.keyCode==231){
            if(window.confirm(`Key 'ç' pressed, do you want to popup-print the temporary trilateration dataset?`)){
                console.log("Popup-printing temporary trilateration dataset")
                window.open('http://victor.scapp.io/print/tri_dataset_temp');
            }
        }

        // change gateway IDs
        if(e.keyCode==103){
            if(window.confirm(`Key 'g' pressed, do you want to change the gateway IDs?`)){
                console.log("Changing gateway IDs");

                // change gateway IDs
                var gatewayR = prompt("Gateway R: ", "004A1092");
                var gatewayG = prompt("Gateway G: ", "004A1093");
                var gatewayB = prompt("Gateway B: ", "004A1526");
                const url='http://victor.scapp.io/param/gateways';
                const data={'gatewayR': gatewayR, 'gatewayG': gatewayG, 'gatewayB': gatewayB}
                axios({method: 'POST', url: url, data: data})
            }
        }

        // kill
        if(e.keyCode==107){
            kill_button_cb(1)
        }

        // hover
        if(e.keyCode==104){
            kill_button_cb(2)
        }

        // return
        if(e.keyCode==114){
            kill_button_cb(3)
        }
    };
}


//#########################################################################################
//###############################  PARAMETERS AND STATE ###################################
//#########################################################################################

// function when parameter is changed
function param_change(){
    // get from HTML elements
    var loop_todo = parseInt(document.getElementById("loop_todo").value);
    var rad1 = parseFloat(document.getElementById("rad1").value);
    var rad2 = parseFloat(document.getElementById("rad2").value);
    var rad3 = parseFloat(document.getElementById("rad3").value);
    var hover = parseFloat(document.getElementById("hover").value);
    var flight = parseFloat(document.getElementById("flight").value);
    var takeoff = parseFloat(document.getElementById("takeoff").value);

    // check validity 
    if(loop_todo<=0){
        window.alert("Loops todo parameter is not correct, should be positive")
        return;
    }
    if(rad1<0 || rad2<0 || rad3<0){
        window.alert("Radius parameters are not correct, should be positive")
        return;
    }
    if(hover<0){
        window.alert("Hover parameters is not correct, should be positive")
        return;
    }
    if(flight<0 || takeoff<0){
        window.alert("Altitudes parameters are not correct, should be positive")
        return;
    }
    if(flight<5 || takeoff<2){
        window.alert("Altitudes parameters are too small")
        return;
    }
    if(flight<=takeoff){
        window.alert("Altitudes parameters are not correct, flight should be above takeoff")
        return;
    }

    // POST on server
    const url='http://victor.scapp.io/param/change_from_maps';
    const data={'rad1': rad1, 'rad2': rad2, 'rad3': rad3, 'hover': hover, 'loop_todo': loop_todo, 'flight': flight, 'takeoff': takeoff};
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
        document.getElementById("rad3").value = response.data['radius']['v3'];
        document.getElementById("hover").value = response.data['hovering time'];
        document.getElementById("flight").value = response.data['altitude']['flying'];
        document.getElementById("takeoff").value = response.data['altitude']['takeoff'];
    });
}

// set the states in GUI
function change_states(stateR, stateG, stateB, altitudeR, altitudeG, altitudeB, batteryR, batteryG, batteryB){
    // change the string in HTML stuff
    document.getElementById('states').children[1].innerHTML = '<img src=marker/droneR.png width=20> State:'+stateR+', altitude:'+altitudeR+', battery:'+batteryR;
    document.getElementById('states').children[2].innerHTML = '<img src=marker/droneG.png width=20> State:'+stateG+', altitude:'+altitudeG+', battery:'+batteryG;
    document.getElementById('states').children[3].innerHTML = '<img src=marker/droneB.png width=20> State:'+stateB+', altitude:'+altitudeB+', battery:'+batteryB;
}

// testing if drone is online
function check_drone_online(markers){

    // check online from server
    const url='http://victor.scapp.io/param/check_online';
    axios({method: 'POST', url: url})
    .catch(function (error){
        if(window.confirm("Server is offline\nOK to try again\n")){
            get_base_param();
            markers.network.setMap(null);
        }
    })
    .then(function(response){
        
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
        if( (droneR_online==true) + (droneG_online==true) + (droneB_online==true) >= 2)
            document.getElementById('all').disabled = false;
        else
            document.getElementById('all').disabled = true;
    });

    // deactivate buttons 
    if(bool_some_drone_is_flying){

        // network
        document.getElementById("network_place").disabled = true;
        document.getElementById("network_get").disabled = true;
        document.getElementById("network_closeby").disabled = true;
        document.getElementById("network_latlng").disabled = true;

        // parameters
        document.getElementById("send2").disabled = true;
        document.getElementById("loop_todo").disabled = true;
        document.getElementById("rad1").disabled = true;
        document.getElementById("rad2").disabled = true;
        document.getElementById("rad3").disabled = true;
        document.getElementById("hover").disabled = true;
        document.getElementById("flight").disabled = true;
        document.getElementById("takeoff").disabled = true;
    }
    else{

        // network
        document.getElementById("network_place").disabled = false;
        document.getElementById("network_get").disabled = false;
        document.getElementById("network_closeby").disabled = false;
        document.getElementById("network_latlng").disabled = false;

        // parameters
        document.getElementById("send2").disabled = false;
        document.getElementById("loop_todo").disabled = false;
        document.getElementById("rad1").disabled = false;
        document.getElementById("rad2").disabled = false;
        document.getElementById("rad3").disabled = false;
        document.getElementById("hover").disabled = false;
        document.getElementById("flight").disabled = false;
        document.getElementById("takeoff").disabled = false;
    }

    // call function again after X seconds
    setTimeout(function(){
        check_drone_online(markers)
    }, delay_online_check);
}

// compute distance between node and est
function compute_result_dist(){

    // define variables
    lat1 = lat_node;
    lon1 = lng_node;
    lat2 = lat_last_est;
    lon2 = lng_last_est;

    // check undefined
    if (typeof lat1=='undefined' || typeof lon1=='undefined' || typeof lat2=='undefined' || typeof lon2=='undefined'){
        return -1;
    }

    // check zero for node (initialization)
    if (lat1==0 || lon1==0){
        return -1;
    }

    // code from https://www.geodatasource.com/developers/javascript
    if ((lat1 == lat2) && (lon1 == lon2)) {
        return 0;
    }
    else {
        var radlat1 = Math.PI * lat1/180;
        var radlat2 = Math.PI * lat2/180;
        var theta = lon1-lon2;
        var radtheta = Math.PI * theta/180;
        var dist = Math.sin(radlat1) * Math.sin(radlat2) + Math.cos(radlat1) * Math.cos(radlat2) * Math.cos(radtheta);
        if (dist > 1) {
            dist = 1;
        }
        dist = Math.acos(dist);
        dist = dist * 180/Math.PI;
        dist = dist * 60 * 1.1515;
        dist = dist * 1.609344 * 1000;  // conversion to meters
        return dist.toFixed(1);
    }
}

// get the results
function print_results(){

    // print results in header
    document.querySelector('.results').innerHTML = 
        "Node:("+lat_node.toFixed(5)+","+lng_node.toFixed(5)+"), est:("+lat_last_est.toFixed(5)+","+lng_last_est.toFixed(5)+"), distance:("+compute_result_dist()+"m)";
}



//#########################################################################################
//################################  MAPS AND CONTROL ######################################
//#########################################################################################

// all the gui elements
function init_gui_elements() {
    init_legend();
    init_states();
    init_takeoff_buttons();
    init_param();
    init_network_param();
    init_safeties();
    init_results();
}

// init show_gui control
function init_show_gui(){
    // create showgui object
    var showgui = document.getElementById('show_gui');

    // push on map
    map.controls[google.maps.ControlPosition.TOP_CENTER].push(showgui);
}

// init centering controls
function init_centering_controls() {

    // create div for centering button
    var centerControlDiv = document.createElement('div');
    var centerControl = new center_control(centerControlDiv, lat_lausanne, lng_lausanne, "Lausanne");
    centerControlDiv.index = 1;
    map.controls[google.maps.ControlPosition.TOP_CENTER].push(centerControlDiv);

    // create div for centering button
    var centerControlDiv2 = document.createElement('div');
    var centerControl2 = new center_control(centerControlDiv2, lat_zurich, lng_zurich, "Zurich");
    centerControlDiv2.index = 1;
    map.controls[google.maps.ControlPosition.TOP_CENTER].push(centerControlDiv2);

    // create div for centering button
    var centerControlDiv3 = document.createElement('div');
    var centerControl3 = new center_control(centerControlDiv3, 0, 0, "Zero");
    centerControlDiv3.index = 1;
    map.controls[google.maps.ControlPosition.TOP_CENTER].push(centerControlDiv3);

    // create div for centering button
    var centerControlDiv4 = document.createElement('div');
    var centerControl4 = new center_control(centerControlDiv4, lat_node, lng_node, "Node");
    centerControlDiv4.index = 1;
    map.controls[google.maps.ControlPosition.TOP_CENTER].push(centerControlDiv4);
}

// creates a GUI for centering map
function center_control(control_div, lat, lng, string){

    // Set CSS for the control border.
    var controlUI = document.createElement('div');
    controlUI.style.backgroundColor = '#fff';
    controlUI.style.border = '2px solid #fff';
    controlUI.style.borderRadius = '3px';
    controlUI.style.boxShadow = '0 2px 6px rgba(0,0,0,.3)';
    controlUI.style.cursor = 'pointer';
    controlUI.style.margin = '3px';
    controlUI.style.textAlign = 'center';
    controlUI.title = 'Click to recenter the map';
    control_div.appendChild(controlUI);

    // Set CSS for the control interior.
    var controlText = document.createElement('div');
    controlText.style.color = '#000';
    controlText.style.fontFamily = 'Roboto,Arial,sans-serif';
    controlText.style.fontSize = '14px';
    controlText.style.lineHeight = '20px';
    controlText.style.paddingLeft = '5px';
    controlText.style.paddingRight = '5px';
    controlText.innerHTML = string;
    controlUI.appendChild(controlText);

    // Setup the click event listeners: simply set the map to Chicago.
    controlUI.addEventListener('click', function() {
        if(string=='Node'){
            lat = lat_node
            lng = lng_node
        }
        console.log("Centering on "+string+", lat:"+lat+", lng:"+lng)
        map.setCenter(new google.maps.LatLng(lat, lng));
        map.setZoom(16);
    });
}

// init states
function init_states(){
    // create states object
    var states = document.getElementById('states');

    // add droneR
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/droneR.png width=20> State:'+stateR+', altitude:'+altitudeR;
    states.appendChild(div);

    // add droneG
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/droneG.png width=20> State:'+stateG+', altitude:'+altitudeG;
    states.appendChild(div);

    // add droneB
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/droneB.png width=20> State:'+stateB+', altitude:'+altitudeB;
    states.appendChild(div);

    // push on map
    map.controls[google.maps.ControlPosition.RIGHT_BOTTOM].push(states);
}

// init param
function init_param(){
    // create param object
    var param = document.getElementById('param');

    // push on map
    map.controls[google.maps.ControlPosition.RIGHT_BOTTOM].push(param);
}

// init legend
function init_legend(){
    // create legend object
    var legend = document.getElementById('legend');

    // add home
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/homeR.png width=20> <img src=marker/homeG.png width=20> <img src=marker/homeB.png width=20> Homes'
    legend.appendChild(div);

    // add drone
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/droneR.png width=20> <img src=marker/droneG.png width=20> <img src=marker/droneB.png width=20> Drones'
    legend.appendChild(div);

    // add waypoint past
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/waypointR_old_circle.png width=20> <img src=marker/waypointG_old_circle.png width=20> <img src=marker/waypointB_old_circle.png width=20> Past waypoints'
    legend.appendChild(div);

    // add waypoint current
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/waypointR_current_circle.png width=20> <img src=marker/waypointG_current_circle.png width=20> <img src=marker/waypointB_current_circle.png width=20> Current waypoints'
    legend.appendChild(div);
    
    // add node
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/node_circle.png width=20> Node ground truth'
    legend.appendChild(div);

    // add network
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/network_circle.png width=20> Network estimate'
    legend.appendChild(div);

    // add estcont
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/estimate_cont_circle.png width=20> Temporary estimation'
    legend.appendChild(div);

    // add est
    var div = document.createElement('div');
    div.innerHTML = '<img src=marker/estimate2_circle.png width=20> <img src=marker/estimate_circle.png width=20> <img src=marker/estimate3_circle.png width=20> Loop estimation'
    legend.appendChild(div);

    // push on map
    map.controls[google.maps.ControlPosition.RIGHT_BOTTOM].push(legend);
}

// init network buttons
function init_network_param(){
    // create param object
    var network_param = document.getElementById('network_param');

    // push on map
    map.controls[google.maps.ControlPosition.LEFT_TOP].push(network_param);
}

// init takeoff buttons
function init_takeoff_buttons() {
    // create takeoff_button object
    var takeoff_button = document.getElementById('takeoff_button');

    // push on map
    map.controls[google.maps.ControlPosition.LEFT_TOP].push(takeoff_button);
}

// init safety buttons
function init_safeties() {
    // create safeties object
    var safeties = document.getElementById('safeties');

    // push on map
    map.controls[google.maps.ControlPosition.LEFT_TOP].push(safeties);
}

// init results
function init_results() {
    // create results object
    var results = document.getElementById('results');

    // push on map
    map.controls[google.maps.ControlPosition.BOTTOM_CENTER].push(results);
}

// get lora network estimate
function get_lora_network_estimate() {

    // get network est from server
    const url='http://victor.scapp.io/lora/compute_network_est';
    axios({method: 'GET', url: url}).then(function(response){
        if(response.data=='No location received...') {
            console.log(response.data)
            window.alert("No location data was ever received by the server, please place network manually");
            network_button_cb('none');
        }
        else{
            console.log(response.data)
            window.alert(response.data)
            //var lat = parseFloat(response.data.substring(4+response.data.indexOf('lat='), response.data.length))
            //var lng = parseFloat(response.data.substring(4+response.data.indexOf('lng='), response.data.length))
            network_button_cb('none');
        }
    })
}




//#########################################################################################
//################################  FIREBASE OBJECTS ######################################
//#########################################################################################

// Firebase for home
function firebase_homes(markers) {

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
        oms.addMarker(markers.homeR);

        // set start as home
        lat_homeR = lat;
        lng_homeR = lng;

        // spider click listener
        google.maps.event.addListener(markers.homeR, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.homeR.title);
            iw.open(map, markers.homeR);
        });
    });
    home_refR.on('child_removed', function(snapshot) {
        markers.homeR.setMap(null)
    });

    // for homeG
    home_refG.on('child_added', function(snapshot){

        // get coordinates
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // move marker
        markers.homeG.setPosition(new google.maps.LatLng(lat, lng));
        oms.addMarker(markers.homeG);

        // set start as home
        lat_homeG = lat;
        lng_homeG = lng;

        // spider click listener
        google.maps.event.addListener(markers.homeG, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.homeG.title);
            iw.open(map, markers.homeG);
        });
    });
    home_refG.on('child_removed', function(snapshot) {
        markers.homeG.setMap(null)
    });

    // for homeB
    home_refB.on('child_added', function(snapshot){

        // get coordinates
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // move marker
        markers.homeB.setPosition(new google.maps.LatLng(lat, lng));
        oms.addMarker(markers.homeB);

        // set start as home
        lng_homeB = lat;
        lng_homeB = lng;

        // spider click listener
        google.maps.event.addListener(markers.homeB, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.homeB.title);
            iw.open(map, markers.homeB);
        });
    });
    home_refB.on('child_removed', function(snapshot) {
        markers.homeB.setMap(null)
    });
}

// Firebase for drones and paths
function firebase_drones(markers, paths) {

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
        oms.addMarker(markers.droneR);

        // draw path
        paths.listR.push({lat: lat, lng: lng});
        paths.lineR.setPath(paths.listR);
        paths.lineR.setMap(map)

        // change state in HTML
        stateR = snapshot.val().state;
        altitudeR = parseInt(snapshot.val().rel_alt);
        batteryR = snapshot.val().battery;
        change_states(stateR, stateG, stateB, altitudeR, altitudeG, altitudeB, batteryR, batteryG, batteryB);

        // change variable
        lat_droneR = lat;
        lng_droneR = lng;

        // spider click listener
        google.maps.event.addListener(markers.droneR, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.droneR.title);
            iw.open(map, markers.droneR);
        });
    });
    drone_refR.on('child_removed', function(snapshot) {
        markers.droneR.setMap(null)
        paths.lineR.setMap(null)
        paths.listR = []
        change_states('UNKNOWN', 'UNKNOWN', 'UNKNOWN', 0, 0, 0, 0, 0, 0);
    });

    // for droneG
    drone_refG.on('child_added', function(snapshot) {

        // get coordinates from firebase
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // move marker
        markers.droneG.setPosition(new google.maps.LatLng(lat, lng));
        oms.addMarker(markers.droneG);

        // draw path
        paths.listG.push({lat: lat, lng: lng});
        paths.lineG.setPath(paths.listG);
        paths.lineG.setMap(map)

        // change state in HTML
        stateG = snapshot.val().state;
        altitudeG = parseInt(snapshot.val().rel_alt);
        batteryG = snapshot.val().battery;
        change_states(stateR, stateG, stateB, altitudeR, altitudeG, altitudeB, batteryR, batteryG, batteryB);

        // change variable
        lat_droneG = lat;
        lng_droneG = lng;

        // spider click listener
        google.maps.event.addListener(markers.droneG, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.droneG.title);
            iw.open(map, markers.droneG);
        });
    });
    drone_refG.on('child_removed', function(snapshot) {
        markers.droneG.setMap(null)
        paths.lineG.setMap(null)
        paths.listG = []
        change_states('UNKNOWN', 'UNKNOWN', 'UNKNOWN', 0, 0, 0, 0, 0, 0);
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
        oms.addMarker(markers.droneB);

        // change state in HTML
        stateB = snapshot.val().state;
        altitudeB = parseInt(snapshot.val().rel_alt);
        batteryB = snapshot.val().battery;
        change_states(stateR, stateG, stateB, altitudeR, altitudeG, altitudeB, batteryR, batteryG, batteryB);

        // change variable
        lat_droneB = lat;
        lng_droneB = lng;

        // spider click listener
        google.maps.event.addListener(markers.droneB, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.droneB.title);
            iw.open(map, markers.droneB);
        });
    });
    drone_refB.on('child_removed', function(snapshot) {
        markers.droneB.setMap(null)
        paths.lineB.setMap(null)
        paths.listB = []
        change_states('UNKNOWN', 'UNKNOWN', 'UNKNOWN', 0, 0, 0, 0, 0, 0);
    });
}

// Firebase for waypoints
function firebase_waypoints(wp_lists) {

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

        // create title for waypoint
        var title;
        if(snapshot.val().id=='WP'){
            title = 'Waypoint R (#'+wp_lists.wpR.length+')';
        }
        else{
            title = 'Waypoint R (#'+wp_lists.wpR.length+','+snapshot.val().id+')';
        }

        // create new marker
        var waypoint_marker = new google.maps.Marker({
            position: {lat: lat, lng: lng},
            map: map,
            icon: 'marker/waypointR_current.png',
            title: title,
        });
        oms.addMarker(waypoint_marker);

        // add it to list
        wp_lists.wpR.push(waypoint_marker);

        // spider click listener
        wp_lists.wpR.forEach(function(item, index, array){
            google.maps.event.addListener(item, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
                iw.setContent(item.title);
                iw.open(map, item);
            });
        });
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

        // create title for waypoint
        var title;
        if(snapshot.val().id=='WP'){
            title = 'Waypoint G (#'+wp_lists.wpG.length+')';
        }
        else{
            title = 'Waypoint G (#'+wp_lists.wpG.length+','+snapshot.val().id+')';
        }

        // create new marker
        var waypoint_marker = new google.maps.Marker({
            position: {lat: lat, lng: lng},
            map: map,
            icon: 'marker/waypointG_current.png',
            title: title,
        });
        oms.addMarker(waypoint_marker);

        // add it to list
        wp_lists.wpG.push(waypoint_marker);

        // spider click listener
        wp_lists.wpG.forEach(function(item, index, array){
            google.maps.event.addListener(item, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
                iw.setContent(item.title);
                iw.open(map, item);
            });
        });
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
            item.setIcon('marker/waypointB_old.png');
        });
        oms.addMarker(waypoint_marker);

        // create title for waypoint
        var title;
        if(snapshot.val().id=='WP'){
            title = 'Waypoint B (#'+wp_lists.wpB.length+')';
        }
        else{
            title = 'Waypoint B (#'+wp_lists.wpB.length+','+snapshot.val().id+')';
        }

        // create new marker
        var waypoint_marker = new google.maps.Marker({
            position: {lat: lat, lng: lng},
            map: map,
            icon: 'marker/waypointB_current.png',
            title: title,
        });

        // add it to list
        wp_lists.wpB.push(waypoint_marker);

        // spider click listener
        wp_lists.wpB.forEach(function(item, index, array){
            google.maps.event.addListener(item, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
                iw.setContent(item.title);
                iw.open(map, item);
            });
        });
    });
    wayp_refB.on('child_removed', function(snapshot) {
        wp_lists.wpB.forEach(function(item, index, array){
            item.setMap(null)
        });  
    });
}

// Firebase for estimations
function firebase_estimations(markers, circles) {

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
        oms.addMarker(markers.network);

        // storage
        lat_last_est = lat;
        lng_last_est = lng;

        // print results
        print_results();

        // spider click listener
        google.maps.event.addListener(markers.network, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.network.title);
            iw.open(map, markers.network);
        });
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
            oms.addMarker(markers.network);
        }
        if(type=='est1') {
            circles.c2.setCenter(new google.maps.LatLng(lat, lng));
            circles.c2.setMap(map);
            circles.c2.setRadius(rad);
            markers.est1.setPosition(new google.maps.LatLng(lat, lng));
            oms.addMarker(markers.est1);
        }
        if(type=='est2') {
            circles.c3.setCenter(new google.maps.LatLng(lat, lng));
            circles.c3.setMap(map);
            circles.c3.setRadius(rad);
            markers.est2.setPosition(new google.maps.LatLng(lat, lng));
            oms.addMarker(markers.est2);
        }
        if(type=='est3') {
            circles.c4.setCenter(new google.maps.LatLng(lat, lng));
            circles.c4.setMap(map);
            circles.c4.setRadius(rad);
            markers.est3.setPosition(new google.maps.LatLng(lat, lng));
            oms.addMarker(markers.est3);
        }
        if(type=='temp') {
            circles.c_cont.setCenter(new google.maps.LatLng(lat, lng));
            circles.c_cont.setMap(map);
            circles.c_cont.setRadius(rad);
            markers.est_cont.setPosition(new google.maps.LatLng(lat, lng));
            oms.addMarker(markers.est_cont);
        }

        // storage
        lat_last_est = lat;
        lng_last_est = lng;

        // print results
        print_results();

        // spider click listener
        google.maps.event.addListener(markers.est1, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.est1.title);
            iw.open(map, markers.est1);
        });
        google.maps.event.addListener(markers.est2, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.est2.title);
            iw.open(map, markers.est2);
        });
        google.maps.event.addListener(markers.est3, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.est3.title);
            iw.open(map, markers.est3);
        });
        google.maps.event.addListener(markers.network, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.network.title);
            iw.open(map, markers.network);
        });
        google.maps.event.addListener(markers.est_cont, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.est_cont.title);
            iw.open(map, markers.est_cont);
        });
    });
    est_ref.on('child_removed', function(snapshot) {
        circles.c1.setMap(null);
        circles.c2.setMap(null);
        circles.c3.setMap(null);
        circles.c4.setMap(null);
        circles.c_cont.setMap(null);
        markers.est1.setMap(null);
        markers.est2.setMap(null);
        markers.est3.setMap(null);
        markers.est_cont.setMap(null);
    });
}   

// Firebase for node
function firebase_node(markers) {

    // reference in Firebase
    var node_ref = firebase.child('node');

    // for network points
    node_ref.on('child_added', function(snapshot) {

        // get coordinates from firebase
        var lat = snapshot.val().lat;
        var lng = snapshot.val().lng;

        // move marker
        markers.node.setPosition(new google.maps.LatLng(lat, lng));
        oms.addMarker(markers.node);

        // storage
        lat_node = lat;
        lng_node = lng;

        // print results
        print_results();

        // spider click listener
        google.maps.event.addListener(markers.node, 'spider_click', function(e) {  // 'spider_click', not plain 'click'
            iw.setContent(markers.node.title);
            iw.open(map, markers.node);
        });
    });
    node_ref.on('child_removed', function(snapshot) {
        markers.node.setMap(null)
    });
}




//#########################################################################################
//################################  BUTTONS CALLBACKS #####################################
//#########################################################################################

// callback for killswitch button
function kill_button_cb(kill){
    if(kill==1){
        console.log("Sending kill command to all drones");
        document.getElementById('unkill').style.backgroundColor=color_can_click;
        document.getElementById('kill').style.backgroundColor='#f55';
        document.getElementById('hold').style.backgroundColor=color_can_click;
        document.getElementById('rtl').style.backgroundColor=color_can_click;

        // reactivate things
        bool_some_drone_is_flying = false;
    }
    else if(kill==0){
        document.getElementById('unkill').style.backgroundColor=color_clicked;
        document.getElementById('kill').style.backgroundColor=color_can_click;
        document.getElementById('hold').style.backgroundColor=color_can_click;
        document.getElementById('rtl').style.backgroundColor=color_can_click;
    }
    else if(kill==2){
        console.log("Sending hover command to all drones");
        document.getElementById('unkill').style.backgroundColor=color_can_click;
        document.getElementById('kill').style.backgroundColor=color_can_click;
        document.getElementById('hold').style.backgroundColor=color_clicked;
        document.getElementById('rtl').style.backgroundColor=color_can_click;
    }
    else if(kill==3){
        console.log("Sending RTL coommand to all drones");
        document.getElementById('unkill').style.backgroundColor=color_can_click;
        document.getElementById('kill').style.backgroundColor=color_can_click;
        document.getElementById('hold').style.backgroundColor=color_can_click;
        document.getElementById('rtl').style.backgroundColor=color_clicked;
    }

    // POST on server
    const url='http://victor.scapp.io/param/drone_kill';
    const data={'kill': kill}
    axios({method: 'POST', url: url, data: data})
}

// callback for takeoff button
function takeoff_button_cb(drone_id) {
    //console.log("Starting drone: "+drone_id)

    // create data according to which button was pressed
    var data;
    if(drone_id=='R') {
        // data to send 
        data={'bool_drone1_start': 1, 'bool_drone2_start': 0, 'bool_drone3_start': 0};

        // buttons color
        document.getElementById('R').style.backgroundColor=color_clicked;
        document.getElementById('G').style.backgroundColor=color_can_click;
        document.getElementById('B').style.backgroundColor=color_can_click;
        document.getElementById('X').style.backgroundColor=color_can_click;
        document.getElementById('all').style.backgroundColor=color_can_click;

        // go back in None after X seconds
        console.log("Setting drone "+drone_id+" to takeoff, disabling network estimate change");
        setTimeout(function(){
            console.log("Setting drone takeoff back to none ("+delay_after_takeoff/1000+" seconds passed)");
            takeoff_button_cb('X')
        }, delay_after_takeoff);

        // disable network estimate change
        network_button_cb('none');

        // deactivate things
        bool_some_drone_is_flying = true;
    }
    if(drone_id=='G') {
        // data to send 
        data={'bool_drone1_start': 0, 'bool_drone2_start': 1, 'bool_drone3_start': 0};

        // buttons color
        document.getElementById('R').style.backgroundColor=color_can_click;
        document.getElementById('G').style.backgroundColor=color_clicked;
        document.getElementById('B').style.backgroundColor=color_can_click;
        document.getElementById('X').style.backgroundColor=color_can_click;
        document.getElementById('all').style.backgroundColor=color_can_click;

        // go back in None after X seconds
        console.log("Setting drone "+drone_id+" to takeoff, disabling network estimate change");
        setTimeout(function(){
            console.log("Setting drone takeoff back to none ("+delay_after_takeoff/1000+" seconds passed)");
            takeoff_button_cb('X')
        }, delay_after_takeoff);

        // disable network estimate change
        network_button_cb('none');

        // deactivate things
        bool_some_drone_is_flying = true;
    }
    if(drone_id=='B') {
        // data to send 
        data={'bool_drone1_start': 0, 'bool_drone2_start': 0, 'bool_drone3_start': 1};

        // buttons color
        document.getElementById('R').style.backgroundColor=color_can_click;
        document.getElementById('G').style.backgroundColor=color_can_click;
        document.getElementById('B').style.backgroundColor=color_clicked;
        document.getElementById('X').style.backgroundColor=color_can_click;
        document.getElementById('all').style.backgroundColor=color_can_click;

        // go back in None after X seconds
        console.log("Setting drone "+drone_id+" to takeoff, disabling network estimate change");
        setTimeout(function(){
            console.log("Setting drone takeoff back to none ("+delay_after_takeoff/1000+" seconds passed)");
            takeoff_button_cb('X')
        }, delay_after_takeoff);

        // disable network estimate change
        network_button_cb('none');

        // deactivate things
        bool_some_drone_is_flying = true;
    }
    if(drone_id=='X') {
        // data to send 
        data={'bool_drone1_start': 0, 'bool_drone2_start': 0, 'bool_drone3_start': 0};

        // buttons color
        document.getElementById('R').style.backgroundColor=color_can_click;
        document.getElementById('G').style.backgroundColor=color_can_click;
        document.getElementById('B').style.backgroundColor=color_can_click;
        document.getElementById('X').style.backgroundColor=color_clicked;
        document.getElementById('all').style.backgroundColor=color_can_click;
    }
    if(drone_id=='all') {
        var confirmation = window.confirm("Are you sure?")

        if(confirmation==true){
            // data to send 
            data={'bool_drone1_start': 1, 'bool_drone2_start': 1, 'bool_drone3_start': 1};

            // buttons color
            document.getElementById('R').style.backgroundColor=color_can_click;
            document.getElementById('G').style.backgroundColor=color_can_click;
            document.getElementById('B').style.backgroundColor=color_can_click;
            document.getElementById('X').style.backgroundColor=color_can_click;
            document.getElementById('all').style.backgroundColor=color_clicked;

            // go back in None after X seconds
            console.log("Setting all drones to takeoff, disabling network estimate change");
            setTimeout(function(){
                console.log("Setting drone takeoff back to none ("+delay_after_takeoff/1000+" seconds passed)");
                takeoff_button_cb('X')
            }, delay_after_takeoff);

            // disable network estimate change
            network_button_cb('none');
        }
        else{
            data={'bool_drone1_start': 0, 'bool_drone2_start': 0, 'bool_drone3_start': 0};
            takeoff_button_cb('X');

            // deactivate things
            bool_some_drone_is_flying = true;
        }
    }

    // POST on server
    const url='http://victor.scapp.io/param/drone_start';
    axios({method: 'POST', url: url, data: data})
}

// callback for network estimate button
function network_button_cb(type) {

    // different types of network estimate 
    if(type=='get') {
        // button color
        document.getElementById('network_get').style.backgroundColor=color_clicked;
        document.getElementById('network_place').style.backgroundColor=color_can_click;
        document.getElementById('network_none').style.backgroundColor=color_can_click;
        document.getElementById('network_closeby').style.backgroundColor=color_can_click;
        document.getElementById('network_latlng').style.backgroundColor=color_can_click;

        // click listener
        click_listener_active = false

        // call function
        get_lora_network_estimate();
    }
    if(type=='place') {
        // button color
        document.getElementById('network_get').style.backgroundColor=color_can_click;
        document.getElementById('network_place').style.backgroundColor=color_clicked;
        document.getElementById('network_none').style.backgroundColor=color_can_click;
        document.getElementById('network_closeby').style.backgroundColor=color_can_click;
        document.getElementById('network_latlng').style.backgroundColor=color_can_click;

        // click listener
        click_listener_active = true
    }
    if(type=='none') {
        // button color
        document.getElementById('network_get').style.backgroundColor=color_can_click;
        document.getElementById('network_place').style.backgroundColor=color_can_click;
        document.getElementById('network_none').style.backgroundColor=color_clicked;
        document.getElementById('network_closeby').style.backgroundColor=color_can_click;
        document.getElementById('network_latlng').style.backgroundColor=color_can_click;

        // click listener
        click_listener_active = false
    }
    if(type=='latlng') {
        // button color
        document.getElementById('network_get').style.backgroundColor=color_can_click;
        document.getElementById('network_place').style.backgroundColor=color_can_click;
        document.getElementById('network_none').style.backgroundColor=color_can_click;
        document.getElementById('network_closeby').style.backgroundColor=color_can_click;
        document.getElementById('network_latlng').style.backgroundColor=color_clicked;

        // click listener
        click_listener_active = false

        // ask user input
        var lat = prompt("Latitude: ", "46.51341")
        var lng = prompt("Longitude: ", "6.356288")

        // post network estimate on server
        const url='http://victor.scapp.io/lora/network_estimate_latlng';
        const data={'lat': lat, 'lng': lng}
        axios({method: 'POST', url: url, data: data})
    }
    if(type=='node') {
        // button color
        document.getElementById('network_get').style.backgroundColor=color_can_click;
        document.getElementById('network_place').style.backgroundColor=color_can_click;
        document.getElementById('network_none').style.backgroundColor=color_can_click;
        document.getElementById('network_closeby').style.backgroundColor=color_clicked;
        document.getElementById('network_latlng').style.backgroundColor=color_can_click;

        // click listener
        click_listener_active = false

        // get max distance from node
        var max_dist = prompt("Maximum distance from node: ", "0");

        // get random things around node
        var dist_meters = Math.random()*max_dist; 
        var angle_degrees = Math.random()*360;
        var answer = shift_latlng(lat_node, lng_node, dist_meters, angle_degrees)
        var lat = answer.lat; var lng = answer.lng;

        // post network estimate on server
        const url='http://victor.scapp.io/lora/network_estimate_latlng';
        const data={'lat': lat, 'lng': lng}
        axios({method: 'POST', url: url, data: data})
    }
}

// callback for gui show checkbox
function gui_show_cb() {
    // get checkbox value
    bool_show_gui = document.getElementById("show_gui_checkbox").checked;
    
    // (de)activate elements
    if(bool_show_gui) {
        var element = document.getElementById("legend");
        element.style.display = "block";
        var element = document.getElementById("states");
        element.style.display = "block";
        var element = document.getElementById("param");
        element.style.display = "block";
        var element = document.getElementById("network_param");
        element.style.display = "block";
        var element = document.getElementById("results");
        element.style.display = "block";
        var element = document.getElementById("takeoff_button");
        element.style.display = "block";
        var element = document.getElementById("safeties");
        element.style.display = "block";
    }
    else{
        var element = document.getElementById("legend");
        element.style.display = "none";
        var element = document.getElementById("states");
        element.style.display = "none";
        var element = document.getElementById("param");
        element.style.display = "none";
        var element = document.getElementById("network_param");
        element.style.display = "none";
        var element = document.getElementById("results");
        element.style.display = "none";
        var element = document.getElementById("takeoff_button");
        element.style.display = "none";
        var element = document.getElementById("safeties");
        element.style.display = "none";
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
function create_markers(icons) {

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
        title: 'First loop estimation',
    });
    var est_marker2 = new google.maps.Marker({
        map: map,
        icon: 'marker/estimate.png',
        title: 'Second loop estimation',
    });
    var est_marker3 = new google.maps.Marker({
        map: map,
        icon: 'marker/estimate3.png',
        title: 'Third loop estimation',
    });
    var est_marker_cont = new google.maps.Marker({
        map: map,
        icon: 'marker/estimate_cont.png',
        title: 'Temporary estimation',
    });

    // for node
    var node_marker = new google.maps.Marker({
        map: map,
        icon: 'marker/node.png',
        title: 'Node ground truth',
    });

    // return them 
    return {droneR: droneR_marker, droneG: droneG_marker, droneB: droneB_marker, homeR: home_markerR, homeG: home_markerG, homeB: home_markerB, network: network_marker, est1: est_marker1, est2: est_marker2, est3: est_marker3, est_cont: est_marker_cont, node: node_marker};
}

// creates the circles
function create_circles() {

    // for network estimations
    var network_circle1 = new google.maps.Circle({
        strokeColor: '#000000',
        strokeOpacity: 0.5,
        strokeWeight: 2,
        fillColor: '#222222',       // for network, grey
        fillOpacity: 0.1,
    })
    var network_circle2 = new google.maps.Circle({
        strokeColor: '#FFBB00',
        strokeOpacity: 0.5,
        strokeWeight: 2,
        fillColor: '#FF9900',       // for est1, orange
        fillOpacity: 0.2,
    })
    var network_circle3 = new google.maps.Circle({
        strokeColor: '#FFFF00',
        strokeOpacity: 0.5,
        strokeWeight: 2,
        fillColor: '#FFFF00',       // for est2, yellow
        fillOpacity: 0.1,
    })
    var network_circle4 = new google.maps.Circle({
        strokeColor: '#00FF00',
        strokeOpacity: 0.5,
        strokeWeight: 2,
        fillColor: '#00FF00',       // for est3, green
        fillOpacity: 0.1,
    })
    var network_circle5 = new google.maps.Circle({
        strokeColor: '#000000',
        strokeOpacity: 0.5,
        strokeWeight: 2,
        fillColor: '#000000',       // for temp est, black
        fillOpacity: 0.15,
    });

    return {c1: network_circle1, c2: network_circle2, c3: network_circle3, c4: network_circle4, c_cont: network_circle5};
}

// creates the waypoints lists (empty)
function create_waypoint_lists() {

    // return empty
    return {wpR: [], wpG: [], wpB: []};
}

// creates the drone paths lines and the list 
function create_paths() { 

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




//#########################################################################################
//##################################  MISC FUNCTIONS ######################################
//#########################################################################################

// adds distance and bearing to latlng
function shift_latlng(lat1, lon1, dist_meters, bearing_degrees){

    var dist = dist_meters / 1000;      // distances in kilometers
    var brng = bearing_degrees * Math.PI / 180;     // angles in radians

    dist = dist / 6371;   

    lat1 = lat1 * Math.PI / 180;
    //lon1 = lon1 * Math.PI / 180;

    var lat2 = Math.asin(Math.sin(lat1) * Math.cos(dist) + Math.cos(lat1) * Math.sin(dist) * Math.cos(brng));
    var lon2 = lon1 + Math.atan2(Math.sin(brng) * Math.sin(dist) * Math.cos(lat1), Math.cos(dist) - Math.sin(lat1) * Math.sin(lat2)) *180 / Math.PI;

    if (isNaN(lat2) || isNaN(lon2)) return null;

    return {lat:lat2*180/Math.PI, lng:lon2};
}