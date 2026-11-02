# x10-ctl - local control for a rooted eufy X10 robot.
# Bare `make` lists targets and changes nothing.

GO      ?= go
BIN     ?= x10-ctl
ROBOT_ARCH ?= arm64
LDFLAGS ?= -s -w

.DEFAULT_GOAL := help

##@ General
.PHONY: help
help: ## List targets
	@awk 'match($$0,/^[a-zA-Z0-9_-]+:.*## /){t=substr($$0,1,index($$0,":")-1);d=substr($$0,index($$0,"## ")+3);printf"  \033[36m%-10s\033[0m %s\n",t,d} /^##@/{printf"\n\033[1m%s\033[0m\n",substr($$0,5)}' $(MAKEFILE_LIST)

##@ Develop
.PHONY: setup
setup: ## Fetch modules (clean clone ready for build/test)
	$(GO) mod download

.PHONY: build
build: ## Build for this host
	$(GO) build -ldflags="$(LDFLAGS)" -trimpath -o $(BIN) ./cmd/x10-ctl

.PHONY: cross
cross: ## Build for the robot (linux/$(ROBOT_ARCH)), no absolute paths
	GOOS=linux GOARCH=$(ROBOT_ARCH) CGO_ENABLED=0 \
		$(GO) build -ldflags="$(LDFLAGS)" -trimpath -o $(BIN)-$(ROBOT_ARCH) ./cmd/x10-ctl

.PHONY: test
test: ## Host unit tests
	$(GO) test ./...

.PHONY: lint
lint: ## go vet
	$(GO) vet ./...

.PHONY: fmt
fmt: ## gofmt the tree
	gofmt -w .

.PHONY: run
run: build ## Build and run against a local robot
	./$(BIN)

.PHONY: clean
clean: ## Remove build outputs
	rm -f $(BIN) $(BIN)-$(ROBOT_ARCH)
