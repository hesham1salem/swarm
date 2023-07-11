package main

// methods
import (
	"bytes"
	"fmt"
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"
)

func handleTasks(c *gin.Context) {
	var req goalTaskReq
	if err := c.ShouldBindJSON(&req); err != nil {

		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}
	res := goalTaskRes{}
	fmt.Printf("req: %v\n", req)
	err := sendGoal(&req, &res)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	}
	//todo  store in database

	c.JSON(http.StatusOK, res)

}

func sendGoal(req *goalTaskReq, res *goalTaskRes) error {
	if node == nil {
		return fmt.Errorf("ROS node is not initialized")
	}
	if goalServiceClient == nil {
		return fmt.Errorf("goalServiceClient is not connected")
	}
	srvRes := &goalTaskRes{}
	err := goalServiceClient.Call(req, srvRes)
	if err != nil {
		return err
	}
	res.Success = srvRes.Success
	return nil
}

func sendRequest(req_ros taskStatusReq) {
	url := "http://127.0.0.1:8080/api/tasks/" + strconv.Itoa(int(req_ros.Task_id))

	req, err := http.NewRequest("POST", url, bytes.NewReader([]byte(fmt.Sprintf("%v", gin.H{"status": req_ros.Status}))))

	if err != nil {
		fmt.Println("Error creating request:", err)
		return
	}

	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Accept", "application/json")

	client := &http.Client{}
	resp, err := client.Do(req)
	if err != nil {
		fmt.Println("Error sending request:", err)
		return
	}
	defer resp.Body.Close()

}

func task_status_callback(req *taskStatusReq) (*taskStatusRes, bool) {
	sendRequest(*req)
	fmt.Println(req)
	return &taskStatusRes{
		Success: true,
	}, true
}
