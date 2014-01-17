#!/usr/bin/perl -w

#
# event_monitor.pl
#
#      Author: gm
#
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
#

use IO::Socket;

# flush after every write
$| = 1;

my $server_port = 5001;
my ($sock, $received_data);
my ($peeraddress,$peerport);

$sock = new IO::Socket::INET( LocalPort => $server_port, Proto     => "udp")
   or die "Couldn't be a udp server on port $server_port : $@\n";
   
print "Lin_guider event monitor\n-------------------\nListening...\n";

while( 1 )
{
	$sock->recv( $received_data, 1024, 0);
  
	$peer_address = $sock->peerhost();
	$peer_port = $sock->peerport();
	print "\n($peer_address:$peer_port) said : $received_data";

#	$data = "received from lin_guider\n";
#	print $sock "$data";
}

$sock->close();
