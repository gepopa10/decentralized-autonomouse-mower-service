const getAccountsResult = document.getElementById('getAccountsResult');
const ethereumButton = document.getElementById('connectWallet');
const StartMissionButton = document.getElementById('StartMission');
const loadingBar = document.getElementById('loadingBar');
const ConnectRobotButton = document.getElementById('ConnectRobot');
const SnapshotButton = document.getElementById('Snapshot');

const sleep = (delay) => new Promise((resolve) => setTimeout(resolve, delay))

const initialize = async () => {

	// Create the main viewer.
	var viewer = new ROS3D.Viewer({
		divID: 'urdf',
		width: 600,
		height: 300,
		antialias: true,
		background: '#ffffff',
		fixedFrame: '/odom'
	});

	// Add a grid.
	var grid = new ROS3D.Grid({
		num_cells: 500,
		color: '#cccccc',
		lineWidth: 1,
		cellSize: 1
	});
	viewer.addObject(grid);

	let ros;

	async function connectROS(robot_url) {
		var url = "wss://" + robot_url
		ros = new ROSLIB.Ros({
			url: url
		});

		ros.on('connection', function() {
			console.log('Connected to websocket server.');
		});

		ros.on('error', function(error) {
			console.log('Error connecting to websocket server: ', error);
			document.getElementById('error').style.display = 'inline';
		});

		ros.on('close', function() {
			console.log('Connection to websocket server closed.');
		});



		var listener_odom = new ROSLIB.Topic({
			ros: ros,
			name: '/odom',
			messageType: 'nav_msgs/Odometry'
		});

		listener_odom.subscribe(function(message) {
			var basePose = new ROSLIB.Pose(message.pose.pose);
			viewer.cameraControls.center.x = basePose.position.x;
			viewer.cameraControls.center.y = basePose.position.y;
			viewer.cameraControls.center.z = basePose.position.z;
		});

		var listener_mission_finished = new ROSLIB.Topic({
			ros: ros,
			name: '/move_base_simple/mission_finished',
			messageType: 'std_msgs/Float64'
		});

		listener_mission_finished.subscribe(function(message) {
			loadingBar.innerHTML = "Mission completed in " + message.data + " minutes"
		});

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
			path: 'https://bafybeia3httvdwijmbvbc2pgqibqebiya34ls3hczitfxs7a7ed7nx3o6y.ipfs.dweb.link/',
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

	}

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

		loadingBar.innerHTML = "----> Executing <----"
	}

	var account_connected = false;

	StartMissionButton.addEventListener('click', async () => {
		if(!account_connected) {
			loadingBar.innerHTML = "----> Failed! Connect Account! <----"
			return;
		}

		var error = await sendTransaction();

		if(error) {
			loadingBar.innerHTML = "----> Transaction failed! <----"
			return;
		} else {
			loadingBar.innerHTML = "----> Transaction succeed! <----"
		}

		await sleep(500);
		printLoadingBar();

		var publish_raw_path_from_fullpath_meters_client = new ROSLIB.Service({
			ros: ros,
			name: '/publish_raw_path_from_fullpath_meters',
			serviceType: 'boustrophedon_optimal_path_planner/publish_raw_path_from_fullpath_meters'
		});

		var request = new ROSLIB.ServiceRequest({
			input_file_name: "fullpath_meters.txt",
			nb_waypoints_path: 4000
		});

		publish_raw_path_from_fullpath_meters_client.callService(request, function(result) {
			console.log('Result for service call on ' +
				publish_raw_path_from_fullpath_meters_client.name +
				': ' +
				result.result);
		});

	});


	let robot_url

	ConnectRobotButton.addEventListener('click', async () => {
		if(!account_connected) {
			loadingBar.innerHTML = "----> Failed! Connect Account! <----"
			return;
		}

		var error = await requestRobotUrl();

		robotContractInstanceAdapter.events.RobotUrlRequestFulfilled()
			.on("data", function(event) {
				robot_url = event.returnValues.robot_url;
				console.log("Got robot url from event:", robot_url);
			}).on("error", console.error);

		while((typeof robot_url == 'undefined') || (robot_url == '')) {

			loadingBar.innerHTML = "----> Waiting to receive confirmation... <----"
			await sleep(500);
			robot_url = await robotContractInstanceAdapter.methods.robot_url().call();
			loadingBar.innerHTML = "Robot Url : " + robot_url
			await sleep(500);
		}

		if(error) {
			loadingBar.innerHTML = "----> Connection to robot failed! <----"
			return;
		} else {
			loadingBar.innerHTML = "----> Connection to robot succeed! <----"
		}

		//"chainlink-robot.diode.link:8100"
		await connectROS(robot_url);

		await sleep(1000);

		loadingBar.innerHTML = "Connected to robot: " + robot_url

		ConnectRobotButton.setAttribute("disabled", null);
		ConnectRobotButton.style.visibility = 'hidden';
	});

	web3 = new Web3(web3.currentProvider);

	let robotContractInstance
	robotContractInstance = new web3.eth.Contract(abi, contractAddress);

	let robotContractInstanceAdapter
	robotContractInstanceAdapter = new web3.eth.Contract(abiAdapter, contractAddressAdapter);

	let account

	ethereumButton.addEventListener('click', () => {
		//Will Start the metamask extension
		getAccount();
	});

	async function getAccount() {
		const accounts = await ethereum.request({ method: 'eth_requestAccounts' });
		if(accounts[0]) {
			getAccountsResult.innerHTML = accounts[0] || 'Not able to get accounts';
			account = accounts[0];
			account_connected = true;
		} else {
			getAccountsResult.innerHTML = 'Not able to get accounts';
			account_connected = false;
		}
	}

	const price = '0.0099';

	async function sendTransaction() {
		var failed = true;
		await robotContractInstance.methods.start_mission().send({ from: account, value: web3.utils.toWei(price, 'ether'), gas: 1000000 },
			(error, result) => {

				if(!error) {
					console.log(result);
					failed = false;
				} else {
					console.log("we have an error", error)
					console.log(error);
				}
			}).catch(err => {
			console.log(err);
		});
		return failed;
	}

	async function requestRobotUrl() {
		var failed = true;
		await robotContractInstanceAdapter.methods.requestRobotUrlBytes().send({ from: account, gas: 1000000 },
			(error, result) => {

				if(!error) {
					console.log(result);
					failed = false;
				} else {
					console.log("we have an error", error)
					console.log(error);
				}
			}).catch(err => {
			console.log(err);
		});
		return failed;
	}

	SnapshotButton.addEventListener('click', async () => {
		if(!account_connected) {
			loadingBar.innerHTML = "----> Failed! Connect Account! <----"
			return;
		}

		await addToken();
	});

	async function addToken() {
		const tokenImage = 'https://bafybeiag6nubujmybqwwc2gl6v233ii6g2ge5my4etat5urc7yfgtzz6au.ipfs.nftstorage.link/';
		const txt = await mintToken(tokenImage).then(notify)
	}

	async function mintToken(_uri) {
		const encodedFunction = web3.eth.abi.encodeFunctionCall({
			name: "mintToken",
			type: "function",
			inputs: [{
				type: 'string',
				name: 'tokenURI'
			}]
		}, [_uri]);

		const transactionParameters = {
			to: nftContractAddress,
			from: account,
			data: encodedFunction
		};
		const txt = await ethereum.request({
			method: 'eth_sendTransaction',
			params: [transactionParameters]
		});
		return txt
	}

	async function notify(_txt) {
		document.getElementById("resultSpace").innerHTML =
			`<input disabled = "true" id="result" type="text" class="form-control" placeholder="Description" aria-label="URL" aria-describedby="basic-addon1" value="Your NFT was minted in transaction ${_txt}">`;
	}
}

window.addEventListener('DOMContentLoaded', initialize)
