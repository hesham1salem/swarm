package main

import (

	"github.com/bluenviron/goroslib/v2"
	
	"github.com/gin-gonic/gin"
)


var node *goroslib.Node
var goalServiceClient *goroslib.ServiceClient
var taskStatusServer *goroslib.ServiceProvider

func main() {
	var err error

	r := gin.Default()
	r.POST("/api/tasks", handleTasks)

	node, err = goroslib.NewNode(goroslib.NodeConf{
		Name:          "go_node",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer node.Close()

	taskStatusServer, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     node,
		Name:     "test_srv",
		Srv:      &taskStatus{},
		Callback: task_status_callback,
	})
	if err != nil {
		panic(err)
	}
	defer taskStatusServer.Close()
	goalServiceClient, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node: node,
		Name: "assign_goal",
		Srv:  &goalTask{},
	})
	if err != nil {
		panic(err)
	}
	defer goalServiceClient.Close()

	if err := r.Run(":8080"); err != nil {
		panic(err)
	}
}

