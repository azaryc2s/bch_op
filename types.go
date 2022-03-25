package op

type Instance struct {
	Name    string `json:"name"`
	Comment string `json:"comment"`
	Type    string `json:"type"`

	Dimension       int         `json:"dimension"`
	DisplayDataType string      `json:"display_data_type"`
	EdgeWeightType  string      `json:"edge_weight_type"`
	Depots          []int       `json:"depots"`
	NodeCoordinates [][]float64 `json:"node_coordinates"`
	EdgeWeights     [][]int     `json:"edge_weights"`
	Prices          []int       `json:"prices"`
	TMax            int         `json:"tmax"`
	TSPLength       int         `json:"tsp_length"`

	Solution *Solution
}

type Solution struct {
	Obj       int   `json:"obj"`
	LBound    int   `json:"lbound"`
	UBound    int   `json:"ubound"`
	Optimal   bool  `json:"optimal"`
	RouteCost int   `json:"route_cost"`
	Route     []int `json:"route"`

	Time    string  `json:"time"`
	System  SysInfo `json:"system"`
	Comment string  `json:"comment"`
}

// SysInfo saves the basic system information
type SysInfo struct {
	Platform string
	CPU      string
	RAM      string
}
