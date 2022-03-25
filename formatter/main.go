package main

import (
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"regexp"
)

func main() {
	if len(os.Args) < 2 {
		log.Printf("No arguments passed!")
		return
	}
	
	fileContent, err := ioutil.ReadFile(os.Args[1])

	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}

	writeBackFile(string(fileContent), os.Args[1])
}
func writeBackFile(fileContent, fileName string) {
	fileContent = sanitizeJsonArrayLineBreaks(fileContent)
	err := ioutil.WriteFile(fileName, []byte(fileContent), 0644)
	if err != nil {
		log.Printf("At %s: %s\n", fileName, err.Error())
		return
	}
}

func sanitizeJsonArrayLineBreaks(json string) string {
	res := fmt.Sprintf("%s",json)
	var numbers = regexp.MustCompile(`\s*([0-9]+),\s+([0-9]+)(,)?`)
	var brackets = regexp.MustCompile(`\[(([0-9]+,)+[0-9]+)\s+\](,?)(\s+)`)
	for ; numbers.MatchString(res); {
		res = numbers.ReplaceAllString(res,"$1,$2$3")
	}
	for ; brackets.MatchString(res); {
		res = brackets.ReplaceAllString(res,"[$1]$3$4")
	}
	return res
}
