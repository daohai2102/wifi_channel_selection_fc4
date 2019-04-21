#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>

#include "SshSession.h"

SshSession::SshSession(){
	fprintf(stderr, "Entering SshSession::SshSession()\n");
	mySshSession = ssh_new();
	if (!mySshSession){
		fprintf(stderr, "Error creating ssh session, exiting ...\n");
		exit(1);
	}
	connected = false;
	fprintf(stderr, "Leaving SshSession::SshSession()\n");
}

SshSession::~SshSession(){
	fprintf(stderr, "Entering SshSession::~SshSession()\n");
	if (connected)
		ssh_disconnect(mySshSession);
	ssh_free(mySshSession);
	fprintf(stderr, "Leaving SshSession::~SshSession()\n");
}

SshSession& SshSession::setPrivateKeyFile(const char *filename){
	strcpy(privateKeyFile, filename);
	return *this;
}

SshSession& SshSession::setUserName(const char *name){
	strcpy(userName, name);
	return *this;
}

SshSession& SshSession::setAddress(const char *addr){
	strcpy(address, addr);
	return *this;
}

SshSession& SshSession::setPort(int p){
	port = p;
	return *this;
}

SshSession& SshSession::setVerbosity(int verbo){
	ssh_options_set(mySshSession, SSH_OPTIONS_LOG_VERBOSITY, &verbo);
	return *this;
}

SshSession& SshSession::connectSsh(){
	ssh_options_set(mySshSession, SSH_OPTIONS_HOST, address);
	ssh_options_set(mySshSession, SSH_OPTIONS_PORT, &port);
	ssh_options_set(mySshSession, SSH_OPTIONS_USER, userName);

	int rc = ssh_connect(mySshSession);
	if (rc != SSH_OK){
		throw SshConnectException(ssh_get_error(mySshSession));
	}
	connected = true;

	rc = ssh_userauth_privatekey_file(mySshSession, NULL, privateKeyFile, NULL);
	if (rc != SSH_AUTH_SUCCESS){
		throw SshUserAuthException(ssh_get_error(mySshSession));
	}

	return *this;
}

ssh_channel SshSession::runCommandAsync(const char *cmd){
	ssh_channel channel = ssh_channel_new(mySshSession);
	if (!channel){
		throw SshNewChannelException(ssh_get_error(mySshSession));
	}

	int rc = ssh_channel_open_session(channel);
	if (rc != SSH_OK){
		ssh_channel_free(channel);
		throw SshChannelOpenSessionException(ssh_get_error(mySshSession));
	}

	rc = ssh_channel_request_exec(channel, cmd);
	if (rc != SSH_OK){
		ssh_channel_close(channel);
		ssh_channel_free(channel);
		throw SshExecCommandException(ssh_get_error(mySshSession));
	}
	
	return channel;
}

std::string SshSession::getChannelBuffer(ssh_channel channel){
	char buffer[256];
	std::string response;
	unsigned int nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
	while (nbytes > 0){
		std::cerr << "read " << nbytes << "bytes" << std::endl;
		response.append(buffer, nbytes);
		nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
	}

	if (nbytes < 0){
		ssh_channel_close(channel);
		ssh_channel_free(channel);
		throw SshChannelReadException(ssh_get_error(mySshSession));
	}

	ssh_channel_send_eof(channel);
	ssh_channel_close(channel);
	ssh_channel_free(channel);
	return response;
}

std::string SshSession::runCommand(const char *cmd){
	ssh_channel channel = runCommandAsync(cmd);
	return getChannelBuffer(channel);
}
