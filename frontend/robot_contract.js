const contractAddress = "0x334833e3e5f4ea1f87b9a526edb4d034d34cc0fc"
const abi = [{
		"anonymous": false,
		"inputs": [{
				"indexed": false,
				"internalType": "address",
				"name": "user",
				"type": "address"
			},
			{
				"indexed": false,
				"internalType": "uint256",
				"name": "amount",
				"type": "uint256"
			},
			{
				"indexed": false,
				"internalType": "uint256",
				"name": "time",
				"type": "uint256"
			}
		],
		"name": "mission_started",
		"type": "event"
	},
	{
		"inputs": [],
		"name": "mission_fee",
		"outputs": [{
			"internalType": "uint256",
			"name": "",
			"type": "uint256"
		}],
		"stateMutability": "view",
		"type": "function"
	},
	{
		"inputs": [],
		"name": "start_mission",
		"outputs": [],
		"stateMutability": "payable",
		"type": "function"
	}
]
