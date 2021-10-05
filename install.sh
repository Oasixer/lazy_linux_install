#!/bin/bash

set -e

EMAIL="kaelan.ms@gmail.com"
USERNAME=k
DOTFILES_REPO="git@github.com:Oasixer/dotfiles.git"
INSTALL_PROGRESS_DIR=/home/$USERNAME/temp/install_progress
DISTRO=ubuntu
UDIR=/home/$USERNAME
DF=$UDIR/dotfiles

echo "$USERNAME"

USAGE="Usage: $(basename $0) [-s stepname] [--no-build] | [--help]

Arguments:
  -h, --help                         show this help message and exit
  -s, --step stepname                ie. --step ros
  -b, --no-build                     don't build a new image
"
usage() {
    printf "$USAGE"
}

step=
re_run=
while [ "$1" != "" ]; do
    case $1 in
        -s | --step )           shift
                                step=$1
                                ;;
        -r | --rerun )          re_run=1
                                ;;
        -h | --help )           usage
                                exit
                                ;;
        * )                     usage
                                exit 1
    esac
    shift
done


# if [[ -z "$version" ]]; then
    # echo "Error: missing version number, use -v <version number>"
    # usage
    # exit 1
# fi

# if [[ "$version" == "1.0.0" ]]; then
    # echo "asiodjaosidjaoisjdoi"
# fi

# if [ -z "$no_build" ]; then
    # echo "no-build option not set"
# fi

remove() {
	rm -f $1
}


# TODO install python versions, venv, flask, etc
step_gate() {
	TARGET=$1
	CHECK_FILE=$INSTALL_PROGRESS_DIR/$TARGET
	sudo -u $USERNAME mkdir -p $INSTALL_PROGRESS_DIR
	if [ ! -f $CHECK_FILE ]; then
		echo "running setup/install step: $TARGET"
		eval "$TARGET" && touch $CHECK_FILE
	else
		echo "skipping step: $TARGET"
	fi
}


apt_stuff () {
	apt update # after running this we know shit is updated
	apt install -y \
		vim \
		neovim \
		curl \
		nodejs \
		zsh \
		git \
		xclip \
		snapd \
		flameshot \
		neofetch \
		autokey-gtk \
		g++ \
		libgtk-3-dev \
		libtool \
		gtk-doc-tools \
		gnutls-bin \
		valac \
		intltool \
		libpcre2-dev \
		libglib3.0-cil-dev \
		libgnutls28-dev \
		libgirepository1.0-dev \
		libxml2-utils \
		gperf \
		wget \
		mitmproxy \
		build-essential \
    apt-transport-https \
    ca-certificates \
    gnupg2 \
    software-properties-common \
    thefuck \
    silversearcher-ag \
		npm
}

trexo () {
	apt install -y \
		picocom \
		openjdk-11-jdk

	snap install mattermost-desktop --beta
	snap install android-studio --classic
	snap install intellij-idea-ultimate --classic
}

install_ros() {
	sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
	apt install ros-noetic-desktop-full
	apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
	rosdep init
	sudo -u $USERNAME rosdep update
	sudo -u $USERNAME mkdir -p $UDIR/catkin_ws/src
	cd $UDIR/catkin_ws
	sudo -u $USERNAME catkin_make
	source /opt/ros/noetic/setup.bash
	apt install ros-noetic-catkin
	source devel/setup.bash
	# sh \\
    # -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \\
        # > /etc/apt/sources.list.d/ros-latest.list'
	# /bin/su -c "-wget http://packages.ros.org/ros.key -O - | sudo apt-key add -" - $USERNAME
}
# ohmyzsh () {
    # sudo -u $USERNAME sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
    # sudo -u $USERNAME git clone https://github.com/zsh-users/zsh-completions ${ZSH_CUSTOM:=/home/$USERNAME/.oh-my-zsh/custom}/plugins/zsh-completions
# }

chrome () {
	sudo -u $USERNAME wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | apt-key add -
	sudo -u $USERNAME sh -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google.list'
	apt install google-chrome-stable -y

	# apt remove firefox-esr -y
}

git () {
	sudo -u $USERNAME ssh-keygen -t ed25519 -C $EMAIL
	eval "$(sudo -u $USERNAME ssh-agent -s)"
	ssh-add /home/$USERNAME/.ssh/id_ed25519
	sudo -u $USERNAME xclip -selection clipboard < /home/$USERNAME/.ssh/id_ed25519.pub
	sudo -u $USERNAME google-chrome-stable https://github.com/settings/ssh/new
	read -n 1 -p "press any letter to continue after adding ssh key to github"

	sudo -u $USERNAME git config --global user.name "Kaelan Moffett-Steinke"
	sudo -u $USERNAME git config --global user.email "kaelan.ms@gmail.com"
}

# birame () {
    # sudo -u $USERNAME git clone https://github.com/maniat1k/birame.git /home/$USERNAME/.oh-my-zsh/custom/themes/birame
    # cp /home/$USERNAME/.oh-my-zsh/custom/themes/birame/birame.zsh-theme /home/$USERNAME/.oh-my-zsh/custom/themes/birame.zsh-theme
# }

# termite () {
# 	sudo -u $USERNAME git clone https://github.com/thestinger/vte-ng.git
# 	sudo -u $USERNAME echo export LIBRARY_PATH="/usr/include/gtk-3.0:$LIBRARY_PATH"
# 
# 	cd vte-ng
# 	sudo -u $USERNAME ./autogen.sh
# 	sudo -u $USERNAME make
# 	make install
# 	sudo -u $USERNAME git clone --recursive https://github.com/thestinger/termite.git
# 	cd termite
# 	sudo -u $USERNAME make
# 	make install
# 	ldconfig
# 	mkdir -p /lib/terminfo/x
# 	ln -s /usr/local/share/terminfo/x/xterm-termite /lib/terminfo/x/xterm-termite
# 	update-alternatives --install /usr/bin/x-terminal-emulator x-terminal-emulator /usr/local/bin/termite 60
# 
# 	exit
# }
# 
# termite_style () {
# 	cd .config
# 	cd termite-style
# 	sudo -u $USERNAME ./install
# 	echo "Running termite-style. Choose theme=material, font=hack"
# 	sudo -u $USERNAME termite-style
# }

snaps () {
    snap install discord
    snap install lotion
    snap install postman
    snap install slack --classic
	# snap install alacritty --classic
}

# fzf () {
    # sudo -u $USERNAME git clone --depth 1 https://github.com/junegunn/fzf.git /home/$USERNAME/.fzf
    # /home/$USERNAME/.fzf/install
# }

install_spotify () {
    # mkdir $UDIR/programs
    # cd $UDIR/programs
    # curl -sS https://download.spotify.com/debian/pubkey_0D811D58.gpg | apt-key add -
    # echo "deb http://repository.spotify.com stable non-free" | tee /etc/apt/sources.list.d/spotify.list
    # apt install spotify-client -y
		snap install spotify
}

dotfiles () {
	# cd /home/$USERNAME
  # sudo -u $USERNAME git init
  # sudo -u $USERNAME git remote add origin $DOTFILES_REPO
	# sudo -u $USERNAME git checkout $BRANCH
	# sudo -u $USERNAME git submodule init
	# sudo -u $USERNAME git submodule update
	# sudo -u $USERNAME git pull
	sudo -u $USERNAME git clone $DOTFILES_REPO $DF
	sudo -u $USERNAME ln -s $DF/.zshrc $UDIR/.zshrc
	sudo -u $USERNAME ln -s $DF/.zshenv $UDIR/.zshenv
	sudo -u $USERNAME ln -s $DF/.Xmodmap $UDIR/.Xmodmap
	sudo -u $USERNAME rm $UDIR/.gitconfig
	sudo -u $USERNAME ln -s $DF/.gitconfig $UDIR/.gitconfig
	sudo -u $USERNAME ln -s $DF/.ideavimrc $UDIR/.ideavimrc
	sudo -u $USERNAME ln -s $DF/.ssh/config $UDIR/.ssh/config
	sudo -u $USERNAME echo "sleep 4 && xmodmap ~/.Xmodmap &" >> $UDIR/.profile
	cd $UDIR
}

docker () {
		if [ $DISTRO == "debian" ]; then
			apt-key fingerprint 0EBFCD88
			sudo -u $USERNAME curl -fsSL https://download.docker.com/linux/debian/gpg | sudo apt-key add -
			add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/debian $(lsb_release -cs) stable"
		elif [ $DISTRO == "ubuntu" ]; then
			sudo -u $USERNAME curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
			add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
		fi

    apt-cache policy docker-ce
    apt-get install docker-ce docker-ce-cli containerd.io -y
    curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    chmod +x /usr/local/bin/docker-compose
    ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose
    # curl -sS https://dl.yarnpkg.com/ubuntu/pubkey.gpg | apt-key add -
    usermod -aG docker $USERNAME
}

yarn_flake () {
    /bin/su -c 'echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list' - $USERNAME
    apt install yarn -y
    apt install flake8 -y
}

nvim () {
  sudo -u $USERNAME mkdir -p /home/$USERNAME/.config
  sudo -u $USERNAME git clone git@github.com:Oasixer/nvim /home/$USERNAME/.config/nvim

}

node () {
	/bin/su -c "curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.38.0/install.sh | bash" - $USERNAME
	/bin/su -c "/bin/zsh -i -c \"nvm install 16.9\"" - $USERNAME
	/bin/su -c "/bin/zsh -i -c \"nvm use 16.9\"" - $USERNAME
}

# install_alacrity () {
	# echo "#!/bin/sh" >> /usr/bin/start-alacritty
	# echo "/usr/bin/snap run alacritty" >> /usr/bin/start-alacritty
	# chmod --reference=/usr/bin/ls /usr/bin/start-alacritty
	# update-alternatives --install /usr/bin/x-terminal-emulator x-terminal-emulator /usr/bin/start-alacritty 50
	# update-alternatives --config x-terminal-emulator
# }

kitty () {
	/bin/su -c "curl -L https://sw.kovidgoyal.net/kitty/installer.sh | sh /dev/stdin" - $USERNAME

	if [ ! -d $UDIR/.local/bin ]; then
		sudo -u $USERNAME mkdir $UDIR/.local/bin
	fi

	# # Create a symbolic link to add kitty to PATH
	sudo -u $USERNAME ln -s $UDIR/.local/kitty.app/bin/kitty $UDIR/.local/bin/

	# # Place the kitty.desktop file somewhere it can be found by the OS
	sudo -u $USERNAME cp $UDIR/.local/kitty.app/share/applications/kitty.desktop $UDIR/.local/share/applications/

	# # Update the path to the kitty icon in the kitty.desktop file
	sudo -u $USERNAME sed -i "s|Icon=kitty|Icon=$UDIR/.local/kitty.app/share/icons/hicolor/256x256/apps/kitty.png|g" $UDIR/.local/share/applications/kitty.desktop
	update-alternatives --install /usr/bin/x-terminal-emulator x-terminal-emulator $UDIR/.local/bin/kitty 50

	sudo -u $USERNAME git clone --depth 1 git@github.com:dexpota/kitty-themes.git $UDIR/.config/kitty/kitty-themes

	sudo -u $USERNAME ln -s $DF/kitty/kitty.conf $UDIR/.config/kitty/kitty.conf 

	sudo -u $USERNAME ln -s $UDIR/.config/kitty/kitty-themes/themes/OneDark.conf $UDIR/.config/kitty/theme.conf
}

zsh_znap() {
	sudo -u $USERNAME mkdir $UDIR/.config/zsh-plugins
	sudo -u $USERNAME git clone --depth 1 -- https://github.com/marlonrichert/zsh-snap.git $UDIR/.config/zsh-plugins/zsh-snap
	/bin/su -c "/bin/zsh -i -c source $UDIR/.config/zsh-plugins/zsh-snap/install.zsh" - $USERNAME
	/bin/su -c "/bin/zsh -i -c source ~/.zshrc" - $USERNAME
	/bin/su -c '/bin/zsh -i -c "znap pull"' - $USERNAME
	/bin/su -c "chsh -s $(which zsh)" - $USERNAME
}

notes() {
	sudo -u $USERNAME git clone git@github.com:Oasixer/notes.git $UDIR/notes
}

java() {
	apt install -y openjdk-11-jdk

	# https://stackoverflow.com/questions/24641536/how-to-set-java-home-in-linux-for-all-users?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
	if [ ! -f /etc/profile.d/java_home.sh ]; then
		echo 'export JAVA_HOME=$(readlink -f /usr/bin/java | sed "s:/bin/java::")' >> /etc/profile.d/java_home.sh
		chmod +x /etc/profile.d/java_home.sh
	fi
}

# if [ $DISTRO == "debian" ]; then
	# if [ ! -f $INSTALL_PROGRESS_DIR/sudo ]; then
		# install_sudoers
	# else
		# echo "skipping sudo"
	# fi
# fi

autokey() {
	# apt install -y autokey-gtk
	# sudo -u $USERNAME git clone git@github.com:Oasixer/autokey_config.git $UDIR/.config/autokey_config

	# replace config dir
	# sudo -u $USERNAME sed -i "3s/.*/    \"userCodeDir\": \"\/home\/$USERNAME\/.config\/autokey_config\/data\",/" $UDIR/.config/autokey/autokey.json

	AUTOKEY_DIR=$UDIR/.config/autokey_config

	folders=""

	for d in $AUTOKEY_DIR/*/ ; do
		folders="${folders}, \"${d::-1}\""
	done
	folders_line="    \"folders\": [${folders:2}],"
	echo $folders_line


	lineNum="$(grep -n '"folders"' $UDIR/.config/autokey/autokey.json | head -n 1 | cut -d: -f1)"
	sudo -u $USERNAME sed -i "${lineNum}s#.*#${folders_line}#" $UDIR/.config/autokey/autokey.json
}

if [[ -z "$step" ]]; then
	step_gate apt_stuff
	#step_gate ohmyzsh
	step_gate chrome
	step_gate git
	step_gate nvim
	step_gate dotfiles
	#step_gate birame
	#step_gate termite
	#step_gate termite_style
	#step_gate fzf
	step_gate yarn_flake
	step_gate snaps
	step_gate node
	step_gate kitty
	step_gate docker
	step_gate notes
	step_gate zsh_znap
	step_gate trexo
	step_gate java
	step_gate autokey
	step_gate node
else
	if [[ "$re_run" == "1" ]]; then
		remove $INSTALL_PROGRESS_DIR/$step
	fi
	step_gate $step
fi
