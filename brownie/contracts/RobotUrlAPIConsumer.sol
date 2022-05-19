//SPDX-License-Identifier: MIT
pragma solidity ^0.8.7;

import "@chainlink/contracts/src/v0.8/ChainlinkClient.sol";

/**
 * Request testnet LINK and ETH here: https://faucets.chain.link/
 * Find information on LINK Token Contracts and get the latest ETH and LINK faucets here: https://docs.chain.link/docs/link-token-contracts/
 */

/**
 * @notice DO NOT USE THIS CODE IN PRODUCTION. This is an example contract.
 */
 //from GenericLargeResponse
contract RobotUrlAPIConsumer is ChainlinkClient {
  using Chainlink for Chainlink.Request;

  // variable bytes returned in a signle oracle response
  bytes public data;
  string public robot_url;
  bytes public image_data;
  string public robot_image_uri;

  /**
   * @notice Initialize the link token and target oracle
   * @dev The oracle address must be an Operator contract for multiword response
   *
   *
   * Kovan Testnet details:
   * Link Token: 0xa36085F69e2889c224210F603D836748e7dC0088
   * Oracle: 0xc57B33452b4F7BB189bB5AfaE9cc4aBa1f7a4FD8 (Chainlink DevRel)
   *
   */
  constructor(address link, address operator_contract) 
  {
    setChainlinkToken(link);
    setChainlinkOracle(operator_contract);
  }

  /**
   * @notice Request variable bytes from the oracle
   */
  function requestRobotUrlBytes(
  )
    public
  {
    data = "";
    robot_url = "";
    bytes32 specId = "24f1447aa11e4a178b73d9fedd12755c";
    uint256 payment = 100000000000000000;
    Chainlink.Request memory req = buildChainlinkRequest(specId, address(this), this.fulfillBytes.selector);
    req.add("get", "https://chainlink-robot.diode.link:8050/get_robot_url");
    req.add("path", "robot_url");
    sendOperatorRequest(req, payment);
  }

  function requestRobotImageUriBytes(
  )
    public
  {
    image_data = "";
    robot_image_uri = "";
    bytes32 specId = "24f1447aa11e4a178b73d9fedd12755c";
    uint256 payment = 100000000000000000;
    Chainlink.Request memory req = buildChainlinkRequest(specId, address(this), this.fulfillRobotImageUriBytes.selector);
    req.add("get", "https://chainlink-robot.diode.link:8050/get_robot_image_uri");
    req.add("path", "cid");
    sendOperatorRequest(req, payment);
  }

  event RequestFulfilled(
    bytes32 indexed requestId,
    bytes indexed data
  );

  event RobotUrlRequestFulfilled(
    bytes32 indexed requestId,
    string robot_url
  );

  event RequestFulfilledImageUri(
    bytes32 indexed requestId,
    bytes indexed data
  );

  event RobotImageUriRequestFulfilled(
    bytes32 indexed requestId,
    string robot_image_uri
  );

  /**
   * @notice Fulfillment function for variable bytes
   * @dev This is called by the oracle. recordChainlinkFulfillment must be used.
   */
  function fulfillBytes(
    bytes32 requestId,
    bytes memory bytesData
  )
    public
    recordChainlinkFulfillment(requestId)
  {
    emit RequestFulfilled(requestId, bytesData);
    data = bytesData;
    robot_url = string(data);
    emit RobotUrlRequestFulfilled(requestId, robot_url);
  }

    function fulfillRobotImageUriBytes(
    bytes32 requestId,
    bytes memory bytesData
  )
    public
    recordChainlinkFulfillment(requestId)
  {
    emit RequestFulfilledImageUri(requestId, bytesData);
    image_data = bytesData;
    robot_image_uri = string(image_data);
    emit RobotImageUriRequestFulfilled(requestId, robot_image_uri);
  }
}