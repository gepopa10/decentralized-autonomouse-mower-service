const contractAddressAdapter = "0x5C5b5320B552f1C0326019c6901DF724D2dc8946"
const abiAdapter = [{
		"inputs": [{
				"internalType": "address",
				"name": "link",
				"type": "address"
			},
			{
				"internalType": "address",
				"name": "operator_contract",
				"type": "address"
			}
		],
		"stateMutability": "nonpayable",
		"type": "constructor",
		"name": "constructor"
	},
	{
		"anonymous": false,
		"inputs": [{
			"indexed": true,
			"internalType": "bytes32",
			"name": "id",
			"type": "bytes32"
		}],
		"name": "ChainlinkCancelled",
		"type": "event"
	},
	{
		"anonymous": false,
		"inputs": [{
			"indexed": true,
			"internalType": "bytes32",
			"name": "id",
			"type": "bytes32"
		}],
		"name": "ChainlinkFulfilled",
		"type": "event"
	},
	{
		"anonymous": false,
		"inputs": [{
			"indexed": true,
			"internalType": "bytes32",
			"name": "id",
			"type": "bytes32"
		}],
		"name": "ChainlinkRequested",
		"type": "event"
	},
	{
		"anonymous": false,
		"inputs": [{
				"indexed": true,
				"internalType": "bytes32",
				"name": "requestId",
				"type": "bytes32"
			},
			{
				"indexed": true,
				"internalType": "bytes",
				"name": "data",
				"type": "bytes"
			}
		],
		"name": "RequestFulfilled",
		"type": "event"
	},
	{
		"anonymous": false,
		"inputs": [{
				"indexed": true,
				"internalType": "bytes32",
				"name": "requestId",
				"type": "bytes32"
			},
			{
				"indexed": false,
				"internalType": "string",
				"name": "robot_url",
				"type": "string"
			}
		],
		"name": "RobotUrlRequestFulfilled",
		"type": "event"
	},
	{
		"inputs": [],
		"name": "data",
		"outputs": [{
			"internalType": "bytes",
			"name": "",
			"type": "bytes"
		}],
		"stateMutability": "view",
		"type": "function"
	},
	{
		"inputs": [{
				"internalType": "bytes32",
				"name": "requestId",
				"type": "bytes32"
			},
			{
				"internalType": "bytes",
				"name": "bytesData",
				"type": "bytes"
			}
		],
		"name": "fulfillBytes",
		"outputs": [],
		"stateMutability": "nonpayable",
		"type": "function"
	},
	{
		"inputs": [],
		"name": "requestRobotUrlBytes",
		"outputs": [],
		"stateMutability": "nonpayable",
		"type": "function"
	},
	{
		"inputs": [],
		"name": "robot_url",
		"outputs": [{
			"internalType": "string",
			"name": "",
			"type": "string"
		}],
		"stateMutability": "view",
		"type": "function"
	}
]
