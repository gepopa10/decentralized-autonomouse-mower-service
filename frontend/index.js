const ethereumButton = document.getElementById('connectWallet');
const sendCoordinatesButton = document.getElementById('sendCoordinates');
const loadingBar = document.getElementById('loadingBar');
const coordinates = document.getElementById('perimeterExtremities');

const sleep = (delay) => new Promise((resolve) => setTimeout(resolve, delay))

const initialize = async () => {

	var ros = new ROSLIB.Ros({
		url: 'ws://localhost:9091'
	});

	ros.on('connection', function() {
		console.log('Connected to websocket server.');
	});

	ros.on('error', function(error) {
		console.log('Error connecting to websocket server: ', error);
	});

	ros.on('close', function() {
		console.log('Connection to websocket server closed.');
	});

	async function printLoadingBar() {
		var template = "Working: "
		var toPrint
		var numTokens = 25
		for(var i = 0; i <= numTokens; i++) {
			toPrint = template
			for(var j = 0; j < i; j++) toPrint += "#"
			for(j = 0; j < numTokens - i; j++) toPrint += "_"
			loadingBar.innerHTML = toPrint
			await sleep(500)
		}

		loadingBar.innerHTML = "----> Finished <----"
		await sleep(15000)
		loadingBar.innerHTML = ""
	}

	sendCoordinatesButton.addEventListener('click', () => {
		console.log(coordinates.value);
    printLoadingBar();

    var publish_raw_path_from_fullpath_meters_client = new ROSLIB.Service({
      ros: ros,
      name: '/publish_raw_path_from_fullpath_meters',
      serviceType: 'boustrophedon_optimal_path_planner/publish_raw_path_from_fullpath_meters'
    });

    var request = new ROSLIB.ServiceRequest({
      input_file_name: "fullpath_meters.txt",
      nb_waypoints_path: parseInt(coordinates.value)
    });

    publish_raw_path_from_fullpath_meters_client.callService(request, function(result) {
      console.log('Result for service call on ' +
        publish_raw_path_from_fullpath_meters_client.name +
        ': ' +
        result.result);
    });

	});

}

window.addEventListener('DOMContentLoaded', initialize)
