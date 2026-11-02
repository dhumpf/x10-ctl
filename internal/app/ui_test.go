package app

// The UI is one big string literal, so nothing compiles it and nothing catches
// a mistake in it. A duplicate `const` shipped once and took the whole page
// down - every handler in the script stopped running, and the status card sat
// on "checking..." forever with a perfectly healthy API behind it. Nothing in
// the Go build, the tests or the deploy noticed.
//
// These tests are the cheapest guard against that class: parse the page the
// server actually serves, and refuse the obvious ways it can be dead on
// arrival.

import (
	"os/exec"
	"regexp"
	"strings"
	"testing"
)

// pageScript returns the contents of the page's <script> block.
func pageScript(t *testing.T) string {
	t.Helper()
	start := strings.Index(pageHTML, "<script>")
	end := strings.LastIndex(pageHTML, "</script>")
	if start < 0 || end < 0 || end <= start {
		t.Fatal("no <script> block in the index page")
	}
	return pageHTML[start+len("<script>") : end]
}

// TestPageScriptNoDuplicateDeclarations catches the exact failure that shipped:
// two `const DAYS=` at the top level, which is a SyntaxError for the whole
// script rather than for the one statement.
func TestPageScriptNoDuplicateDeclarations(t *testing.T) {
	js := pageScript(t)
	// Top level only - an indented declaration is inside a function or block
	// and may legitimately repeat a name.
	decl := regexp.MustCompile(`(?m)^(?:const|let|function)\s+([A-Za-z_$][\w$]*)`)
	seen := map[string]bool{}
	for _, m := range decl.FindAllStringSubmatch(js, -1) {
		name := m[1]
		if seen[name] {
			t.Errorf("%q is declared twice at the top level of the page script; "+
				"that is a SyntaxError and kills every handler on the page", name)
		}
		seen[name] = true
	}
}

// TestPageScriptBalanced is a coarse check that the literal has not been cut in
// half by an edit - the string is assembled by hand and a stray backtick would
// end the Go raw literal, not the JS.
func TestPageScriptBalanced(t *testing.T) {
	js := pageScript(t)
	for _, pair := range []struct {
		name       string
		open, shut rune
	}{{"braces", '{', '}'}, {"parens", '(', ')'}, {"brackets", '[', ']'}} {
		var depth int
		for _, r := range js {
			switch r {
			case pair.open:
				depth++
			case pair.shut:
				depth--
			}
		}
		if depth != 0 {
			t.Errorf("page script %s are unbalanced by %d", pair.name, depth)
		}
	}
	if strings.Contains(js, "`") {
		t.Error("page script contains a backtick, which ends the Go raw string literal early")
	}
}

// TestPageScriptParses runs a real parser when one is available. Skipped rather
// than failed when node is absent, so the suite still runs on the robot.
func TestPageScriptParses(t *testing.T) {
	node, err := exec.LookPath("node")
	if err != nil {
		t.Skip("node not available; relying on the structural checks above")
	}
	cmd := exec.Command(node, "--check", "-")
	cmd.Stdin = strings.NewReader(pageScript(t))
	if out, err := cmd.CombinedOutput(); err != nil {
		t.Errorf("page script does not parse: %v\n%s", err, out)
	}
}

// TestPageReferencesItsHandlers guards the other half of the same problem: an
// element the script wires up by id, renamed or removed in the markup, fails
// silently at load.
func TestPageReferencesItsHandlers(t *testing.T) {
	js := pageScript(t)
	getByID := regexp.MustCompile(`getElementById\('([^']+)'\)`)
	for _, m := range getByID.FindAllStringSubmatch(js, -1) {
		id := m[1]
		if !strings.Contains(pageHTML, `id="`+id+`"`) {
			t.Errorf("script wires up #%s but the markup has no such element", id)
		}
	}
}
