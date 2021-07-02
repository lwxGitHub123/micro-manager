#include "DENetwork.h"
#include "DEExceptions.h"

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>

using namespace std;
using boost::asio::ip::tcp;
using boost::asio::deadline_timer;
using boost::posix_time::seconds;

DENetwork* DENetwork::instance = NULL;

DENetwork* DENetwork::getInstance()
{
	static boost::mutex instanceMutex;

	boost::lock_guard<boost::mutex> lock(instanceMutex);

	if (DENetwork::instance == NULL)
		DENetwork::instance = new DENetwork();

	return DENetwork::instance;
}

DENetwork::DENetwork()
{
	this->read = new tcp::socket(io_service);
	this->write = new tcp::socket(io_service);
	this->connected = false;

	// Sets whether denetwork is operating in debug or release mode.
	// if denetwork is operating in debug mode, then the commands will no longer 
	// be affected by timeouts.
	#ifdef _DEBUG
	this->debugMode = true;
	#else
	this->debugMode = false;
	#endif
}

DENetwork::~DENetwork()
{
	delete this->read;
	delete this->write;
}

bool DENetwork::connect(const char* ip, port portRead, port portWrite)
{
	assert(ip != NULL);
	
	if (this->connected) return true;

	bool successVal = true;
	successVal = ( successVal && this->createSocket(ip, portRead, this->read));
	successVal = ( successVal && this->createSocket(ip, portWrite, this->write));
	this->connected = successVal;
	if (!successVal)
	{
		this->close();
	}

	return successVal;
}

bool DENetwork::close()
{
	this->read->close();
	this->write->close();
	this->connected = false;

	return true;
}


void DENetwork::setResult(optional<boost::system::error_code>* a, const boost::system::error_code& b, std::size_t /*bytes_transferred*/)
{
	if (b != boost::asio::error::operation_aborted)
		*a = b;
}

bool DENetwork::send(void* data, std::size_t size, unsigned long timeout)
{
	using namespace boost; 

	optional<boost::system::error_code> timeout_result;
	optional<boost::system::error_code> write_result;

	deadline_timer timer(this->write->get_io_service());

	timer.expires_from_now(seconds(timeout));
	if (!debugMode)
		timer.async_wait(boost::bind(&DENetwork::setResult, this, &timeout_result, _1, 0));
	async_write(
		*(this->write), 
		boost::asio::buffer((byte*)data, size), 
		boost::bind(&DENetwork::setResult, this, &write_result, boost::asio::placeholders::error,
		 		boost::asio::placeholders::bytes_transferred));

	this->write->get_io_service().reset();
	while ( this->write->get_io_service().run_one() )
	{
		// Normal result.
		if (write_result)
		{
	 		timer.cancel();
		}
		// Time out
		if (timeout_result)
		{	 
			// On Windows xp systems, cancel causes an operation_not_supported exception
			// to be thrown.
			boost::system::error_code ec;
#pragma warning(push)
#pragma warning(disable: 4996)
	 		this->write->cancel(ec);
#pragma warning(pop)
			break;
		}
	}
	
	if (timeout_result)
	{
		BOOST_THROW_EXCEPTION(CommunicationException() << errorMessage("Timeout occurred during send."));
	}

	this->error = *write_result;

	return	(this->error == boost::system::errc::success);
}

bool DENetwork::receive(void* data, std::size_t size, unsigned long timeout)
{
	using namespace boost; 

	optional<boost::system::error_code> timeout_result;
	optional<boost::system::error_code> read_result;

	deadline_timer timer(this->read->get_io_service());
	timer.expires_from_now(seconds(timeout));
	if (!debugMode)
		timer.async_wait(boost::bind(&DENetwork::setResult, this, &timeout_result, _1, 0));
	async_read(
		*(this->read), 
		boost::asio::buffer((byte*)data, size), 
		boost::bind(&DENetwork::setResult, this, &read_result, boost::asio::placeholders::error,
		 		boost::asio::placeholders::bytes_transferred));

	this->read->get_io_service().reset();

	while ( this->read->get_io_service().run_one() )
	{
		// Normal result.
		if (read_result)
		{
	 		timer.cancel();
		}
		// Time out
		if (timeout_result)
		{	
			// On Windows xp systems, cancel causes an operation_not_supported exception
			// to be thrown.
			boost::system::error_code ec;
#pragma warning(push)
#pragma warning(disable: 4996)
	 		this->read->cancel(ec);
#pragma warning(pop)
			break;
		}
	}
	
	if (timeout_result)
	{
		BOOST_THROW_EXCEPTION(CommunicationException() << errorMessage("Timeout occurred during receive."));
	}

	this->error = *read_result;
	return (this->error == boost::system::errc::success);
}

bool DENetwork::createSocket(const char* ip, port no, tcp::socket* socket_)
{
	tcp::endpoint endpoint_(boost::asio::ip::address_v4::from_string(ip), 
							no);
	socket_->connect(endpoint_, this->error);
	int lastError = error.value();
	socket_->set_option(boost::asio::ip::tcp::no_delay(true));

	return (lastError == boost::system::errc::success);
}


long DENetwork::run(long (*command)(), unsigned long /*timeout*/)
{
	return command();
}

string DENetwork::getLastError()
{
	return (this->error.message());
}

boost::mutex* DENetwork::getTransactionMutex()
{
	return &(this->_transactionMutex);
}
