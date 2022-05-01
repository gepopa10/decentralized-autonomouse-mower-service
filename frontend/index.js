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

	// Create the main viewer.
	var viewer = new ROS3D.Viewer({
		divID: 'urdf',
		width: 600,
		height: 300,
		antialias: true,
		background: '#ffffff',
		fixedFrame: '/odom'
	});

	var listener = new ROSLIB.Topic({
		ros: ros,
		name: '/odom',
		messageType: 'nav_msgs/Odometry'
	});

	listener.subscribe(function(message) {
		var basePose = new ROSLIB.Pose(message.pose.pose);
		viewer.cameraControls.center.x = basePose.position.x;
		viewer.cameraControls.center.y = basePose.position.y;
		viewer.cameraControls.center.z = basePose.position.z;
	});

	// Add a grid.
	var grid = new ROS3D.Grid({
		num_cells: 500,
		color: '#cccccc',
		lineWidth: 1,
		cellSize: 1
	});
	viewer.addObject(grid);

	// Setup a client to listen to TFs.
	var tfClient = new ROSLIB.TFClient({
		ros: ros,
		angularThres: 0.01,
		transThres: 0.01,
		rate: 60.0,
		fixedFrame: '/odom'
	});

	// Setup the URDF client.
	var urdfClient = new ROS3D.UrdfClient({
		ros: ros,
		tfClient: tfClient,
		path: 'http://localhost:80/',
		rootObject: viewer.scene,
		loader: ROS3D.COLLADA_LOADER
	});

	// Setup the marker clients.
	var markerClientFootprints = new ROS3D.MarkerClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/move_base_simple/marker/footprints',
		rootObject: viewer.scene
	});
	var markerClientPath = new ROS3D.MarkerClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/move_base_simple/marker/path',
		rootObject: viewer.scene
	});
	var markerClientPathRaw = new ROS3D.MarkerClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/move_base_simple/marker/path_raw',
		rootObject: viewer.scene
	});
	var markerClientNextGoals = new ROS3D.MarkerClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/move_base_simple/marker/next_goals',
		rootObject: viewer.scene
	});
	var markerClientPerimeter = new ROS3D.MarkerClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/move_base_simple/marker/perimeter',
		rootObject: viewer.scene
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
