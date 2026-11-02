package app

import (
	"regexp"
	"strings"
	"testing"
)

// The schedule editor called querySelector('[data-rooms]') on markup that never
// emitted a data-rooms attribute. querySelector returned null, assigning
// .onclick to it threw, and everything after it in that loop - the skip and
// delete handlers - silently never got wired.
//
// The failure looks like "the room picker is missing and two other buttons are
// dead", which is a long way from "a selector matches nothing".
func TestPageSelectorsHaveMarkup(t *testing.T) {
	sel := regexp.MustCompile(`querySelector(?:All)?\('\[(data-[a-z-]+)\]'\)`)
	seen := map[string]bool{}
	for _, m := range sel.FindAllStringSubmatch(pageHTML, -1) {
		attr := m[1]
		if seen[attr] {
			continue
		}
		seen[attr] = true
		if !strings.Contains(pageHTML, attr+`="`) {
			t.Errorf("querySelector('[%s]') has no markup emitting %s=\"...\"", attr, attr)
		}
	}
	if len(seen) == 0 {
		t.Fatal("no data- selectors found; this test has stopped testing anything")
	}
}

// Same class, one level up: an id looked up with getElementById has to be
// emitted somewhere. Ids built by concatenation are skipped - they cannot be
// checked statically and are the reason this is a separate, narrower test.
func TestPageStaticIDsExist(t *testing.T) {
	get := regexp.MustCompile(`getElementById\('([a-zA-Z][a-zA-Z0-9_-]*)'\)`)
	seen := map[string]bool{}
	for _, m := range get.FindAllStringSubmatch(pageHTML, -1) {
		id := m[1]
		if seen[id] {
			continue
		}
		seen[id] = true
		if !strings.Contains(pageHTML, `id="`+id+`"`) {
			t.Errorf("getElementById(%q) but no element carries id=%q", id, id)
		}
	}
	if len(seen) < 10 {
		t.Fatalf("only %d ids found; this test has stopped testing anything", len(seen))
	}
}
