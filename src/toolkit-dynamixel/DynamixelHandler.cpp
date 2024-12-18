#include "DynamixelHandler.h"

DynamixelHandler::DynamixelHandler():
	m_sDeviceName(""), m_fProtocolVersion(0.0), m_i32BaudRate(0),
	m_pPacketHandler(nullptr), m_pPortHandler(nullptr),
	m_bIsDeviceNameSet(false), m_bIsProtocolVersionSet(false), m_bIsPortOpened(false), m_bIsBaudRateSet(false),
	m_ui8DxlError(0), m_i32DxlCommunicationResult(COMM_TX_FAIL)
{
	
}

DynamixelHandler::~DynamixelHandler()
{
	
}

bool DynamixelHandler::openPort()
{
	if (m_pPortHandler == nullptr)
	{
		std::cout << "[ERROR](DynamixelHandler::openPort) m_pPortHandler is null!" << std::endl;
		m_bIsPortOpened = false;
		return m_bIsPortOpened;
	}
	
	if (!m_bIsDeviceNameSet)
	{
		std::cout << "[ERROR](DynamixelHandler::openPort) m_sDeviceName is not set!" << std::endl;
		m_bIsPortOpened = false;
		return m_bIsPortOpened;
	}
	
	if (m_bIsPortOpened)
	{
		std::cout << "[WARNING](DynamixelHandler::openPort) port is already opened!" << std::endl;
		return m_bIsPortOpened;
	}
	
	if (m_pPortHandler->openPort())
	{
		std::cout << "[INFO](DynamixelHandler::openPort) Succeeded to open the port!" << std::endl;
		m_bIsPortOpened = true;
	}
	else
	{
		std::cout << "[ERROR](DynamixelHandler::openPort) Failed to open the port!" << std::endl;
		m_bIsPortOpened = false;
	}
	return m_bIsPortOpened;
}

void DynamixelHandler::closePort()
{
	if (m_pPortHandler == nullptr)
	{
		std::cout << "[ERROR](DynamixelHandler::closePort) m_pPortHandler is null!" << std::endl;
		m_bIsPortOpened = false;
		return;
	}
	
	if (!m_bIsPortOpened)
	{
		std::cout << "[WARNING](DynamixelHandler::openPort) port is already closed!" << std::endl;
		return;
	}
	
	m_pPortHandler->closePort();
	
	std::cout << "[INFO](DynamixelHandler::closePort) Succeeded to close the port!" << std::endl;
	m_bIsPortOpened = false;
}

bool DynamixelHandler::setBaudRate(int i32BaudRate)
{
	m_i32BaudRate = i32BaudRate;
		
	if (nullptr != m_pPortHandler)
	{
		if (m_pPortHandler->setBaudRate(m_i32BaudRate))
		{
			std::cout << "[INFO](DynamixelHandler::setBaudRate) Succeeded to change the baudrate!" << std::endl;
			m_bIsBaudRateSet = true;
		}
		else
		{
			std::cout << "[ERROR](DynamixelHandler::setBaudRate) Failed to change the baudrate!" << std::endl;
			m_bIsBaudRateSet = false;
		}
	}
	else
	{
		std::cout << "[ERROR](DynamixelHandler::setBaudRate) m_pPortHandler is null!" << std::endl;
		m_bIsBaudRateSet = false;
	}
	return m_bIsBaudRateSet;
}

void DynamixelHandler::setDeviceName(std::string sDeviceName)
{
	m_sDeviceName = sDeviceName;
	m_bIsDeviceNameSet = true;
	
	if (nullptr != m_pPortHandler)
	{
		delete m_pPortHandler;
		m_pPortHandler = nullptr;
	}
	
	// Initialize PortHandler instance
	m_pPortHandler = dynamixel::PortHandler::getPortHandler(m_sDeviceName.c_str());
}

void DynamixelHandler::setProtocolVersion(float fProtocolVersion)
{
	m_fProtocolVersion = fProtocolVersion;
	m_bIsProtocolVersionSet = true;
	
	if (nullptr != m_pPacketHandler)
	{
		delete m_pPacketHandler;
		m_pPacketHandler = nullptr;
	}
	
	m_pPacketHandler = dynamixel::PacketHandler::getPacketHandler(m_fProtocolVersion);
}

bool DynamixelHandler::readCurrentJointPosition(std::vector<uint16_t>& vCurrentJointPosition)
{
	bool bIsReadSuccessfull = false;

	for (unsigned int l_joint = 0; l_joint < NB_JOINTS; l_joint++)
	{
		int dxl_comm_result = COMM_TX_FAIL;             // Communication result
		uint8_t dxl_error = 0;
		uint16_t dxl_present_position = 0;

		dxl_comm_result = m_pPacketHandler->read2ByteTxRx(m_pPortHandler, l_joint + 1, ADDR_XL320_PRESENT_POSITION, &dxl_present_position, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			//std::cout << "[ERROR] " << m_pPacketHandler->getTxRxResult(dxl_comm_result) << std::endl;
			bIsReadSuccessfull = false;
		}
		else if (dxl_error != 0)
		{
			//std::cout << "[ERROR] " << m_pPacketHandler->getRxPacketError(dxl_error) << std::endl;
			bIsReadSuccessfull = false;
		}
		else
		{
			vCurrentJointPosition.push_back(dxl_present_position);
			bIsReadSuccessfull = true;
		}
	}

	return bIsReadSuccessfull;
}


bool DynamixelHandler::readCurrentJointTorque(std::vector<uint16_t>& vCurrentJointTorque)
{
	bool bIsReadSuccessfull = false;

	for (unsigned int l_joint = 0; l_joint < NB_JOINTS; l_joint++)
	{
		int dxl_comm_result = COMM_TX_FAIL;             // Communication result
		uint8_t dxl_error = 0;
		uint16_t dxl_present_torque = 0;

		dxl_comm_result = m_pPacketHandler->read2ByteTxRx(m_pPortHandler, l_joint + 1, ADDR_XL320_PRESENT_LOAD, &dxl_present_torque, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			//std::cout << "[ERROR] " << m_pPacketHandler->getTxRxResult(dxl_comm_result) << std::endl;
			bIsReadSuccessfull = false;
		}
		else if (dxl_error != 0)
		{
			//std::cout << "[ERROR] " << m_pPacketHandler->getRxPacketError(dxl_error) << std::endl;
			bIsReadSuccessfull = false;
		}
		else
		{
			vCurrentJointTorque.push_back(dxl_present_torque);
			bIsReadSuccessfull = true;
		}
	}

	return bIsReadSuccessfull;
}


bool DynamixelHandler::sendTargetJointPosition(std::vector<uint16_t>& vTargetJointPosition)
{
	bool bIsSendSuccessfull = false;

	// checks if the vector size is correct
	if (vTargetJointPosition.size() != NB_JOINTS)
	{
		std::cout << "[ERROR] (DynamixelHandler::sendTargetJointPosition): Size of command vector is not correct: " << vTargetJointPosition.size() << " instead of " << NB_JOINTS << "!" << std::endl;
		bIsSendSuccessfull = false;
	}



	for (unsigned int l_joint = 0; l_joint < NB_JOINTS; l_joint++)
	{
		int dxl_comm_result = COMM_TX_FAIL;             // Communication result
		uint8_t dxl_error = 0;
		uint16_t dxl_present_position = 0;
		dxl_comm_result = m_pPacketHandler->write2ByteTxRx(m_pPortHandler, l_joint + 1, ADDR_XL320_GOAL_POSITION, vTargetJointPosition[l_joint], &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			//std::cout << "[ERROR] (DynamixelHandler::sendTargetJointPosition): " << m_pPacketHandler->getTxRxResult(dxl_comm_result) << std::endl;
			bIsSendSuccessfull = false;
		}
		else if (dxl_error != 0)
		{
			//std::cout << "[ERROR] (DynamixelHandler::sendTargetJointPosition): " << m_pPacketHandler->getRxPacketError(dxl_error) << std::endl;
			bIsSendSuccessfull = false;
		}
		else
		{
			bIsSendSuccessfull = true;
		}
	}
	return bIsSendSuccessfull;

}
 
bool DynamixelHandler::enableTorque(bool bEnableTorque)
{
	bool bIsSendSuccessfull = false;
	
	for (unsigned int l_joint = 0; l_joint < NB_JOINTS; l_joint++)
	{
		int dxl_comm_result = COMM_TX_FAIL;             // Communication result
		uint8_t dxl_error = 0;
		
		dxl_comm_result = m_pPacketHandler->write1ByteTxRx(m_pPortHandler, l_joint + 1, ADDR_XL320_TORQUE_ENABLE, bEnableTorque, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS) 
		{
			//std::cout << "[ERROR] (DynamixelHandler::enableTorque): " << m_pPacketHandler->getTxRxResult(dxl_comm_result) << std::endl;
			bIsSendSuccessfull = false;
		}
		else if (dxl_error != 0) 
		{
			//std::cout << "[ERROR] (DynamixelHandler::enableTorque): " << m_pPacketHandler->getRxPacketError(dxl_error) << std::endl;
			bIsSendSuccessfull = false;
		}
		else 
		{
			bIsSendSuccessfull = true;
		}
	}
	return bIsSendSuccessfull;
}

