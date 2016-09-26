#!/usr/bin/perl
# lg_tool.pl
#
#      Author: Rumen G. Bogdanovski <rumen@skyarchive.org>
#
# This file is part of Lin_guider.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

use strict;
use Getopt::Std;
use IO::Socket;
use File::Basename;

my $VERSION = "0.4";
my $verbose = 0;

my %command_val = (
	'GET_VER' => 1,
	'SET_GUIDER_SQUARE_POS' => 2,
	'SAVE_FRAME' => 3,
	'DITHER' => 4,
	'DITHER_NO_WAIT_XY' => 5,
	'GET_DISTANCE' => 6,
	'SAVE_FRAME_DECORATED' => 7,
	'GUIDER' => 8,
	'GET_GUIDER_STATE' => 9,
	'SET_GUIDER_OVLS_POS' => 10,
	'SET_GUIDER_RETICLE_POS' => 11,
	'FIND_STAR' => 12,
	'SET_DITHERING_RANGE' => 13,
	'GET_RA_DEC_DRIFT' => 14
);
my %command_name = reverse %command_val;

use constant HDR_LEN => 8;
#use constant SOCK_NAME => 'tcp://127.0.0.1:5656';
use constant SOCK_NAME => '/tmp/lg_ss';
my $sock_name;


sub print_help() {
	my $N = basename($0);
	print "\n".
	      "Lin-guider command tool v.$VERSION.\n".
	      "This is a GPL software, created by Rumen G.Bogdanovski.\n".
	      "\n".
	      "Usage: $N get_ver [-v]\n".
	      "       $N guider [-v] start|stop\n".
	      "       $N status [-v]\n".
	      "       $N dither [-v]\n".
	      "       $N dither_no_wait [-v] rX rY\n".
	      "       $N get_distance [-v]\n".
	      "       $N get_ra_dec_drift [-v]\n".
	      "       $N set_dithering_range [-v] rage\n".
	      "       $N set_square_pos [-v] X Y\n".
	      "       $N set_ovls_pos [-v] X Y\n".
	      "       $N set_reticle_pos [-v] X Y\n".
	      "       $N find_star [-v]\n".
	      "       $N save_frame [-v] filename\n".
	      "       $N save_frame_decorated [-v] filename\n".
	      "options:\n".
	      "       -v verbose output\n".
	      "       -h print this help\n\n".
	      "       LG_SOCKET environment variable can be set to change the default.\n".
	      "       The default is '".SOCK_NAME."'. To use TCP connection use\n".
	      "       format 'tcp://hostname:port'.\n\n";
}


#
# Communication routines
#
sub lg_connect($) {
	my ($addr) = @_;

	my $sock;
	if ($addr =~ /^tcp:\/\//) {
		$addr =~ s/^tcp:\/\///;
		$sock = IO::Socket::INET->new($addr) or return undef;
	} else {
		$addr =~ s/^unix:\/\///; # Acept "unix://" prefix, not mandatory
		$sock = IO::Socket::UNIX->new($addr) or return undef;
	}
	return $sock;
}


sub lg_disconnect($) {
	my ($sock) = @_;
	close($sock);
}


sub lg_send_cmd($$$) {
	my ($sock, $cmd, $param) = @_;

	my $len = length $param;

	my $cmd = pack('SSL', 2, $cmd, $len);
	if(HDR_LEN != $sock->send($cmd)) {
		return undef;
	}

	if (($len > 0) and ($len != $sock->send($param))) {
		return undef;
	}

	return 1;
}


sub lg_recv_resp($) {
	my ($sock) = @_;

	my $resp;

	$sock->recv($resp, HDR_LEN);
	if (HDR_LEN != length($resp)) {
		return undef;
	}

	my($wrp, $cmd, $len) = unpack ('SSL', $resp);
	if ($len > 0) {
		$sock->recv($resp, $len);
		if ($len != length($resp)) {
			return undef;
		}
        }

	return ($resp, $cmd);
}


sub lg_chat($$) {
	my ($cmd, $paramstr) = @_;

	my $resp;
	my $timeout = 60;  # Timeout for send command and receive relpy

	my $sock = lg_connect($sock_name) or die "Connect to $sock_name: $!\n";
	eval {
		local $SIG{ALRM} = sub { die "Error: can not set timeout\n" };
		alarm($timeout);
		if (! lg_send_cmd($sock, $cmd ,$paramstr)) {
			print STDERR "Send command error.\n";
			lg_disconnect($sock);
			alarm(0);
			return undef;
		}
		($resp, $cmd) = lg_recv_resp($sock);
		lg_disconnect($sock);
		alarm(0);
	};
	if ($@) {
		return ("Error: Operation Timed Out", $cmd);
	}
	return ($resp, $cmd);
}


#
# Lin-guider Commands
#

sub guider {
	my @params = @_;
	if ($#params != 0) {
		print STDERR "guider: Wrong parameters.\n";
		return undef;
	}
	my ($resp,$cmd) = lg_chat($command_val{GUIDER}, $params[0]);
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^OK/) {
		return 1;
	} else {
		return undef;
	}
}


sub status {
	my @params = @_;
	if ($#params >= 0) {
		print STDERR "status: Wrong parameters.\n";
		return undef;
	}
	my ($resp,$cmd) = lg_chat($command_val{GET_GUIDER_STATE} ,"");
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^OK/) {
		return 1;
	} else {
		return undef;
	}
}


sub dither {
	my @params = @_;
	if ($#params >= 0) {
		print STDERR "dither: Wrong parameters.\n";
		return undef;
	}
	my ($resp,$cmd) = lg_chat($command_val{DITHER} ,"");
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^OK/) {
		return 1;
	} else {
		return undef;
	}
}


sub get_ver {
	my @params = @_;
	if ($#params >= 0) {
		print STDERR "get_ver: Wrong parameters.\n";
		return undef;
	}
	my ($resp,$cmd) = lg_chat($command_val{GET_VER} ,"");
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^v./) {
		return 1;
	} else {
		return undef;
	}
}


sub get_distance {
	my @params = @_;
	if ($#params >= 0) {
		print STDERR "get_distance: Wrong parameters.\n";
		return undef;
	}
	my ($resp,$cmd) = lg_chat($command_val{GET_DISTANCE} ,"");
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^Error/) {
		return undef;
	} else {
		return 1;
	}
}


sub get_ra_dec_drift {
	my @params = @_;
	if ($#params >= 0) {
		print STDERR "get_ra_dec_drift: Wrong parameters.\n";
		return undef;
	}
	my ($resp,$cmd) = lg_chat($command_val{GET_RA_DEC_DRIFT} ,"");
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^Error/) {
		return undef;
	} else {
		return 1;
	}
}


sub dither_no_wait {
	my @params = @_;
	if ($#params != 1) {
		print STDERR "dither_no_wait: Wrong parameters.\n";
		return undef;
	}
	my $paramstr = sprintf("%.2f %.2f", $params[0], $params[1]);
	my ($resp,$cmd) = lg_chat($command_val{DITHER_NO_WAIT_XY}, $paramstr);
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^OK/) {
		return 1;
	} else {
		return undef;
	}
}


sub set_square_pos {
	my @params = @_;
	if ($#params != 1) {
		print STDERR "set_sqare_pos: Wrong parameters.\n";
		return undef;
	}
	my $paramstr = sprintf("%.2f %.2f", $params[0], $params[1]);
	my ($resp,$cmd) = lg_chat($command_val{SET_GUIDER_OVLS_POS}, $paramstr);
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^OK/) {
		return 1;
	} else {
		return undef;
	}
}


sub set_reticle_pos {
	my @params = @_;
	if ($#params != 1) {
		print STDERR "set_sqare_pos: Wrong parameters.\n";
		return undef;
	}
	my $paramstr = sprintf("%.2f %.2f", $params[0], $params[1]);
	my ($resp,$cmd) = lg_chat($command_val{SET_GUIDER_RETICLE_POS}, $paramstr);
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^OK/) {
		return 1;
	} else {
		return undef;
	}
}


sub find_star {
	my @params = @_;
	if ($#params >= 0) {
		print STDERR "find_star: Wrong parameters.\n";
		return undef;
	}
	my ($resp,$cmd) = lg_chat($command_val{FIND_STAR} ,"");
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^Error/) {
		return undef;
	} else {
		return 1;
	}
}


sub save_frame {
	my @params = @_;
	if ($#params != 0) {
		print STDERR "save_frame: Wrong parameters.\n";
		return undef;
	}
	my $paramstr = sprintf("%s", $params[0]);
	my ($resp,$cmd) = lg_chat($command_val{SAVE_FRAME}, $paramstr);
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^SAVED/) {
		return 1;
	} else {
		return undef;
	}
}

sub save_frame_decorated {
	my @params = @_;
	if ($#params != 0) {
		print STDERR "save_frame_decorated: Wrong parameters.\n";
		return undef;
	}
	my $paramstr = sprintf("%s", $params[0]);
	my ($resp,$cmd) = lg_chat($command_val{SAVE_FRAME_DECORATED}, $paramstr);
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^SAVED/) {
		return 1;
	} else {
		return undef;
	}
}


sub set_dithering_range {
	my @params = @_;
	if ($#params != 0) {
		print STDERR "set_dithering_range: Wrong parameters.\n";
		return undef;
	}
	my $paramstr = sprintf("%.2f", $params[0]);
	my ($resp,$cmd) = lg_chat($command_val{SET_DITHERING_RANGE}, $paramstr);
	print "$command_name{$cmd} -> $resp\n";
	if ($resp =~ /^OK/) {
		return 1;
	} else {
		return undef;
	}
}


# main routine
sub main {
	my %options = ();

	my $command = shift @ARGV;

	$sock_name = SOCK_NAME;
	if (defined $ENV{LG_SOCKET}) {
		$sock_name = $ENV{LG_SOCKET};
	}

	if (getopts("vh", \%options) == undef) {
		exit 2;
	}

	if (defined($options{h}) or (!defined($command))) {
		print_help();
		exit 0;
	}

	if(defined($options{v})) {
		$verbose = 1;
	}

	if ($command eq "-v") {  # allow -v before cmmand
		$verbose = 1;
		$command = shift @ARGV;
	}

	if ($command eq "get_ver") {
		if (! get_ver(@ARGV)) {
			$verbose && print STDERR "Get info returned error.\n";
			exit 1;
		}
		$verbose && print "Get info succeeded.\n";
		exit 0;

	} elsif ($command eq "guider") {
		if (! guider(@ARGV)) {
			$verbose && print STDERR "GUIDER returned error.\n";
			exit 1;
		}
		$verbose && print "GUIDER succeeded.\n";
		exit 0;

	} elsif ($command eq "status") {
		if (! status(@ARGV)) {
			$verbose && print STDERR "GET_GUIDER_STATE returned error.\n";
			exit 1;
		}
		$verbose && print "GET_GUIDER_STATE succeeded.\n";
		exit 0;

	} elsif ($command eq "set_dithering_range") {
		if (! set_dithering_range(@ARGV)) {
			$verbose && print STDERR "SET_DITHERING_RANGE returned error.\n";
			exit 1;
		}
		$verbose && print "FIND_STAR succeeded.\n";
		exit 0;

	} elsif ($command eq "dither") {
		if (! dither(@ARGV)) {
			$verbose && print STDERR "DITHER returned error.\n";
			exit 1;
		}
		$verbose && print "DITHER succeeded.\n";
		exit 0;

	}  elsif ($command eq "dither_no_wait") {
		if (! dither_no_wait(@ARGV)) {
			$verbose && print STDERR "DITHER_NO_WAIT returned error.\n";
			exit 1;
		}
		$verbose && print "DITHER_NO_WAIT succeeded.\n";
		exit 0;

	} elsif ($command eq "get_distance") {
		if (! get_distance(@ARGV)) {
			$verbose && print STDERR "GET_DISTANCE returned error.\n";
			exit 1;
		}
		$verbose && print "GET_DISTANCE succeeded.\n";
		exit 0;

	} elsif ($command eq "get_ra_dec_drift") {
		if (! get_ra_dec_drift(@ARGV)) {
			$verbose && print STDERR "GET_RA_DEC_DRIFT returned error.\n";
			exit 1;
		}
		$verbose && print "GET_RA_DEC_DRIFT succeeded.\n";
		exit 0;

	} elsif ($command eq "set_square_pos") {
		if (! set_square_pos(@ARGV)) {
			$verbose && print STDERR "SET_GUIDER_SQUARE_POS returned error.\n";
			exit 1;
		}
		$verbose && print "SET_GUIDER_SQUARE_POS succeeded.\n";
		exit 0;

	} elsif ($command eq "set_ovls_pos") {
		if (! set_square_pos(@ARGV)) {
			$verbose && print STDERR "SET_VISIBLE_OVLS_POS returned error.\n";
			exit 1;
		}
		$verbose && print "SET_GUIDER_SQUARE_POS succeeded.\n";
		exit 0;

	} elsif ($command eq "set_reticle_pos") {
		if (! set_reticle_pos(@ARGV)) {
			$verbose && print STDERR "SET_RETICLE_POS returned error.\n";
			exit 1;
		}
		$verbose && print "SET_GUIDER_SQUARE_POS succeeded.\n";
		exit 0;

	} elsif ($command eq "find_star") {
		if (! find_star(@ARGV)) {
			$verbose && print STDERR "FIND_STAR returned error.\n";
			exit 1;
		}
		$verbose && print "FIND_STAR succeeded.\n";
		exit 0;

	} elsif ($command eq "save_frame") {
		if (! save_frame(@ARGV)) {
			$verbose && print STDERR "SAVE_FRAME returned error.\n";
			exit 1;
		}
		$verbose && print "SAVE_FRAME succeeded.\n";
		exit 0;

	} elsif ($command eq "save_frame_decorated") {
		if (! save_frame_decorated(@ARGV)) {
			$verbose && print STDERR "SAVE_FRAME_DECORATED returned error.\n";
			exit 1;
		}
		$verbose && print "SAVE_FRAME_DECRATED succeeded.\n";
		exit 0;

	} elsif ($command eq "-h") {
		print_help();
		exit 0;

	} else {
		print STDERR "There is no such command \"$command\".\n";
		exit 3;
	}
} main;
