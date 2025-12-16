#!/bin/bash

set -e

case "$(uname -s)" in
    Darwin)
        if [ ! -x "$(command -v brew)" ]; then
            /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
        fi
        ;;
    Linux)
        ;;
    *)
        echo "Unsupported operating system."
        exit 1
esac

if [ ! -x "$(command -v docker)" ]; then
    case "$(uname -s)" in
        Darwin)
            brew install docker
            ;;
        Linux)
            mkdir -p bin
            curl -fsSL https://get.docker.com -o bin/get-docker.sh
            sudo sh bin/get-docker.sh
            ;;
        *)
            echo "Docker is not installed. Please install it."
            exit 1
    esac
fi

if [ ! -x "$(command -v act)" ]; then
    case "$(uname -s)" in
        Darwin)
            brew install act
            ;;
        Linux)
            curl -sSL https://raw.githubusercontent.com/nektos/act/master/install.sh | sh
            ;;
        *)
            echo "act is not installed. Please install it."
            exit 1
    esac
fi

sudo ./bin/act push -P ubuntu-latest=ghcr.io/catthehacker/ubuntu:full-latest
