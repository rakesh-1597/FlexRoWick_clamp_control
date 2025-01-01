#include <iostream>
#include <opc/ua/client/client.h>
#include <opc/common/logger.h>

int main() {
    std::cout << "Welcome!"<<std::endl;
    auto logger = spdlog::stderr_color_mt("client");
    std::string endpoint = "opc.tcp://127.0.0.1:4840/freeopcua/server/";
    try {
        // Create an OPC UA client
        std::cout << "1st line in try"<<std::endl;
        OpcUa::UaClient client(logger);  // Attempt to create the client object

        
        std::cout << "trying to connect..."<<std::endl;
        // Connect to the OPC UA server
        client.Connect(endpoint);
        std::cout << "connected!"<<std::endl;
        // Read a variable (example NodeId)
        auto value = client.GetNode("ns=2;i=2");
        if (!value.IsValid()){
            std::cout << "Unable to get the node with specified node Id";
        }
        OpcUa::NodeId nodeId = value.GetId();

        // Print the namespace index and identifier
        std::cout << "NodeId Namespace Index: " << nodeId.GetNamespaceIndex() << std::endl;
        if (nodeId.IsInteger()) {
            std::cout << "Node Identifier (Numeric): " << nodeId.GetIntegerIdentifier() << std::endl;
        } else if (nodeId.IsString()) {
            std::cout << "Node Identifier (String): " << nodeId.GetStringIdentifier() << std::endl;
        } else if (nodeId.IsGuid()) {
            std::cout << "Node Identifier (GUID): " << nodeId.GetGuidIdentifier() << std::endl;
        } else {
            std::cout << "Unknown Node Identifier Type" << std::endl;
        }
        // Disconnect from the server
        client.Disconnect();
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
