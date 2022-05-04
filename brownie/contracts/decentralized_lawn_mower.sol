pragma solidity 0.8.7;

contract decentralized_lawn_mower 
{
    event mission_started(address user, uint amount, uint time);

    uint public mission_fee = 0.0099 ether;

    function start_mission() public payable
    {
        require(msg.value == mission_fee);
        emit mission_started(msg.sender, msg.value, block.timestamp);
    }
}