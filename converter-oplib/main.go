package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"git.solver4all.com/azaryc2s/op"
	"git.solver4all.com/azaryc2s/op/tsp"
	"io/ioutil"
	"log"
	"math"
	"os"
	"regexp"
	"strconv"
	"strings"
)

var scaling float64

func main() {
	var (
		err error
	)
	targetDir := os.Args[1]
	files, err := ioutil.ReadDir(targetDir)
	if err != nil {
		log.Fatal(err)
	}
	var calcTSP string
	if len(os.Args) > 2 {
		calcTSP = os.Args[2]
	}

FILES:
	for _, f := range files {
		if !strings.Contains(f.Name(), ".oplib") {
			continue
		}
		fileName := targetDir + "/" + f.Name()
		fmt.Println(fileName)
		file, err := os.Open(fileName)
		defer file.Close()
		fileName = strings.ReplaceAll(fileName, ".oplib", ".json")

		var name, comment, problemType, edgeWeightType string
		var tmax int
		var depots []int

		scanner := bufio.NewScanner(file)
		line := -1
		nodeCount := 0
		var edgeWeights [][]int
		var coordinates [][]float64
		var nodeScores []int
		var metaData bool
		var nodeCoordSection, depotSection, nodeScoreSection bool
		metaData = true
		for scanner.Scan() {
			line = line + 1
			t := scanner.Text()
			if metaData {
				t = strings.Trim(t," ")
				if t == "NODE_COORD_SECTION" {
					metaData = false
					nodeCoordSection = true
					continue
				}
				lineSplit := strings.Split(t, ":")
				lineSplit[0] = strings.Trim(lineSplit[0], " ")
				lineSplit[1] = strings.Trim(lineSplit[1], " ")

				if lineSplit[0] == "NAME" {
					name = lineSplit[1]
					continue
				}
				if lineSplit[0] == "COMMENT" {
					comment = lineSplit[1]
					continue
				}
				if lineSplit[0] == "TYPE" {
					problemType = lineSplit[1]
					continue
				}
				if lineSplit[0] == "DIMENSION" {
					continue
				}
				if lineSplit[0] == "COST_LIMIT" {
					tmax, err = strconv.Atoi(lineSplit[1])
					if err != nil {
						fmt.Printf("Couldn't parse the cost limit! Skipping file: %s\n", err.Error())
						continue FILES
					}
					continue
				}
				if lineSplit[0] == "EDGE_WEIGHT_TYPE" {
					edgeWeightType = lineSplit[1]
					if edgeWeightType != "EUC_2D" && edgeWeightType != "CEIL_2D" {
						fmt.Printf("Format other then EUC_2D|CEIL_2D. Skipping for now...\n")
						continue FILES
					}
					continue
				}
			}
			if nodeCoordSection {
				if t == "NODE_SCORE_SECTION" {
					nodeCoordSection = false
					nodeScoreSection = true
					continue
				}
				xyString := strings.Split(t, " ")

				x, err := strconv.ParseFloat(xyString[1], 64)
				if err != nil {
					fmt.Printf("Error parsing coordinate index!: %s", err.Error())
				}
				y, err := strconv.ParseFloat(xyString[2], 64)
				if err != nil {
					fmt.Printf("Error parsing coordinate y!: %s", err.Error())
				}
				xy := []float64{x, y}
				coordinates = append(coordinates, xy)
				nodeCount++
			}
			if nodeScoreSection {
				if t == "DEPOT_SECTION" {
					nodeScoreSection = false
					depotSection = true
					continue
				}
				tString := strings.Split(t, " ")
				score, err := strconv.Atoi(tString[1])
				if err != nil {
					fmt.Printf("Error parsing score!: %s", err.Error())
				}
				nodeScores = append(nodeScores, score)
			}
			if depotSection {
				depot, err := strconv.Atoi(t)
				if err != nil {
					fmt.Printf("Error parsing depot!: %s", err.Error())
				}
				if depot < 0 {
					depotSection = false
					continue
				}
				depots = append(depots, depot-1) //its not 0-indexed in the file, so we subtract 1
			}
		}
		/*var scaledCoords [][]int
		var trimmed bool
		scaledCoords, scaling, trimmed = scaleCoordinatesToInts(coordinates)
		tmax = tmax * int(scaling)*/
		edgeWeights = make([][]int, nodeCount)
		for i := 0; i < nodeCount; i++ {
			edgeWeights[i] = make([]int, nodeCount)
		}
		for i := 0; i < nodeCount; i++ {
			for j := i + 1; j < nodeCount; j++ {
				x := coordinates[i][0] - coordinates[j][0]
				y := coordinates[i][1] - coordinates[j][1]
				var distance int
				if edgeWeightType == "EUC_2D" {
					distance = int(math.Sqrt(math.Pow(x, 2)+math.Pow(y, 2)) + 0.5)
				} else if edgeWeightType == "CEIL_2D" {
					distance = int(math.Ceil(math.Sqrt(math.Pow(x, 2)+math.Pow(y, 2))))
				}
				edgeWeights[i][j] = distance
				edgeWeights[j][i] = distance
			}
		}

		var tspLength int
		if calcTSP == "tsp"{
			_, tspLength = tsp.SolveATSP(edgeWeights)
		} else {
			tspLength = 0
		}

		fileComment := comment
		if scaling > 1.1 {
			fileComment = fmt.Sprintf("%s | coordinates scaled by %d from the original to be integers.", fileComment, int(scaling))
			/*if trimmed {
				fileComment = fmt.Sprintf("%s They were trimmed at the 5th decimal point.", fileComment)
			}*/
		}
		inst := op.Instance{Name: name, Comment: fileComment, Type: problemType, Dimension: nodeCount, DisplayDataType: "COORD_DISPLAY", EdgeWeightType: edgeWeightType, NodeCoordinates: coordinates, TSPLength: tspLength, Prices: nodeScores, TMax: tmax, Depots: depots}

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
	var numbers = regexp.MustCompile(`\s*([-]?[0-9]+(\.[0-9]+)?),\s+([-]?[0-9]+(\.[0-9]+)?)(,)?`)
	var brackets = regexp.MustCompile(`\[(([-]?[0-9]+(\.[0-9]+)?,)+[-]?[0-9]+(\.[0-9]+)?)\s+\](,?)(\s+)`)
	for numbers.MatchString(res) {
		res = numbers.ReplaceAllString(res, "$1,$3$5")
	}
	for brackets.MatchString(res) {
		res = brackets.ReplaceAllString(res, "[$1]$5$6")
	}
	return res
}

func scaleCoordinatesToInts(coords [][]float64) ([][]int, float64, bool) {
	scaledCoords := make([][]int, len(coords))
	places, trimmed := findMaxDecimalPointsInCoords(coords)
	scale := math.Pow(10, float64(places))
	for i := 0; i < len(coords); i++ {
		scaledCoords[i] = []int{int(coords[i][0] * scale), int(coords[i][1] * scale)}
	}
	return scaledCoords, scale, trimmed
}

//if returns true, then the decimal points were more than 5 and had the be trimmed
func findMaxDecimalPointsInCoords(coords [][]float64) (int, bool) {
	max := 0
	for i := 0; i < len(coords); i++ {
		places := len(decimalPortion(coords[i][0]))
		if places > max {
			max = places
		}
		places = len(decimalPortion(coords[i][1]))
		if places > max {
			max = places
		}
	}
	if max > 5 {
		return 5, true
	}
	return max, false
}

func decimalPortion(n float64) string {
	nString := strconv.FormatFloat(n, 'f', -1, 64) // produces 0.xxxx0000
	decimalPlaces := ""
	if strings.Contains(nString, ".") {
		decimalPlaces = strings.Split(nString, ".")[1]
	}
	decimalPlaces = strings.TrimRight(decimalPlaces, "0") // remove trailing 0s
	return decimalPlaces
}
