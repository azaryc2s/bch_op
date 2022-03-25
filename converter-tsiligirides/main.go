package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"git.solver4all.com/azaryc2s/op/tsp"
	"io/ioutil"
	"log"
	"math"
	"os"
	"regexp"
	"strconv"
	"strings"
)

type OPInstance struct {
	Name    string `json:"name"`
	Comment string `json:"comment"`
	Type    string `json:"type"`

	Dimension       int         `json:"dimension"`
	DisplayDataType string      `json:"display_data_type"`
	EdgeWeightType  string      `json:"edge_weight_type"`
	Depots          []int       `json:"depots"`
	NodeCoordinates [][]int     `json:"node_coordinates"`
	EdgeWeights     [][]float64 `json:"edge_weights"`
	Prices          []int       `json:"prices"`
	TMax            int         `json:"tmax"`
	TSPLength       float64     `json:"tsp_length"`
}

var scaling float64

func main() {
	var (
		err error
	)
	targetDir := os.Args[1]
	comment := os.Args[2]
	if len(os.Args) > 3 {
		scalingStr := os.Args[3]
		scaling, err = strconv.ParseFloat(scalingStr, 64)
		if err != nil {
			log.Fatal(err)
			return
		}
		comment = fmt.Sprintf("%s | coordinates scaled by %d from the original to be integers", comment, int(scaling))
	} else {
		scaling = 1.0
	}
	files, err := ioutil.ReadDir(targetDir)
	if err != nil {
		log.Fatal(err)
	}
	for _, f := range files {
		if !strings.Contains(f.Name(), ".txt") {
			continue
		}
		fileName := targetDir + "/" + f.Name()
		fmt.Println(fileName)
		file, err := os.Open(fileName)
		defer file.Close()
		fileName = strings.ReplaceAll(fileName, ".txt", ".json")

		scanner := bufio.NewScanner(file)
		line := -1
		nodeCount := 0
		var edgeWeights [][]float64
		var coordinates [][]int
		var nodeScores []int
		var tmax int
		for scanner.Scan() {
			line = line + 1
			t := scanner.Text()
			xyz := strings.Split(t, "\t")
			if line == 0 {
				maxScore, err := strconv.Atoi(xyz[0])
				if err != nil {
					fmt.Printf("Error parsing tmax!: %s", err.Error())
				}
				tmax = maxScore * int(scaling)
				continue
			}
			x, err := strconv.ParseFloat(xyz[0], 64)
			if err != nil {
				fmt.Printf("Error parsing coordinate x!: %s", err.Error())
			}
			y, err := strconv.ParseFloat(xyz[1], 64)
			if err != nil {
				fmt.Printf("Error parsing coordinate y!: %s", err.Error())
			}
			z, err := strconv.Atoi(xyz[2])
			if err != nil {
				fmt.Printf("Error parsing node score!: %s", err.Error())
			}
			xy := []int{int(x * scaling), int(y * scaling)}
			coordinates = append(coordinates, xy)
			nodeScores = append(nodeScores, z)
			nodeCount++
		}
		edgeWeights = make([][]float64, nodeCount)
		for i := 0; i < nodeCount; i++ {
			edgeWeights[i] = make([]float64, nodeCount)
		}
		for i := 0; i < nodeCount; i++ {
			for j := i + 1; j < nodeCount; j++ {
				x := coordinates[i][0] - coordinates[j][0]
				y := coordinates[i][1] - coordinates[j][1]

				distance := math.Sqrt(float64(x*x+y*y)) //coordinates already scaled
				edgeWeights[i][j] = distance
				edgeWeights[j][i] = distance
			}
		}

		_, tspLength := tsp.SolveATSP(edgeWeights)


		inst := OPInstance{Name: strings.ReplaceAll(f.Name(), ".txt", ""), Comment: comment, Type: "OP", Dimension: nodeCount, DisplayDataType: "COORD_DISPLAY", EdgeWeightType: "EUC_2D", NodeCoordinates: coordinates, TSPLength: tspLength, Prices: nodeScores, TMax: tmax, Depots: []int{0, 1}}

		if err := scanner.Err(); err != nil {
			log.Fatal(err)
			continue
		}

		jsonInst, err := json.MarshalIndent(inst, "", "\t")
		if err != nil {
			log.Fatal(err)
			continue
		}

		jsonInst = []byte(sanitizeJsonArrayLineBreaks(string(jsonInst)))
		err = ioutil.WriteFile(fileName, jsonInst, 0644)
		if err != nil {
			log.Fatal(err)
			continue
		}
	}
}

func sanitizeJsonArrayLineBreaks(json string) string {
	res := fmt.Sprintf("%s", json)
	var numbers = regexp.MustCompile(`\s*([-]?[0-9]+),\s+([-]?[0-9]+)(,)?`)
	var brackets = regexp.MustCompile(`\[(([-]?[0-9]+,)+[-]?[0-9]+)\s+\](,?)(\s+)`)
	for numbers.MatchString(res) {
		res = numbers.ReplaceAllString(res, "$1,$2$3")
	}
	for brackets.MatchString(res) {
		res = brackets.ReplaceAllString(res, "[$1]$3$4")
	}
	return res
}
