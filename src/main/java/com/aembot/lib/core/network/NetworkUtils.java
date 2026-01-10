package com.aembot.lib.core.network;

import edu.wpi.first.wpilibj.DataLogManager;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/** Util class for network-related operations */
public final class NetworkUtils {
  public final class MAC {
    /**
     * Formats a byte array mac address into an actual string and returns it. Returns null if the
     * formatting failed
     *
     * @param macBytes String of bytes that represent a hardware MAC address. Nullable
     * @return {@link String} or {@code null} of the formatted address
     */
    public static final String formatMACAddress(byte[] macBytes) {
      if (macBytes == null) return null;

      // Build colon separated string from bytes. Used to support non-standard ethernet MAC
      // addresses just in case
      StringBuilder sb = new StringBuilder();
      for (int i = 0; i < macBytes.length; i++) {
        // Convert byte to hex value, with ":" if this isn't the last byte.
        sb.append(String.format("%02X%s", macBytes[i], (i < macBytes.length - 1) ? ":" : ""));
      }
      return sb.toString();
    }

    /**
     * Get the MAC address of the RoboRIO's networking interface
     *
     * @return The MAC address of the interface formatted as a colon separated string
     */
    public static final String getMACAddress() {
      // this method exists for error handling. For actual retrieval see _getMACAdress
      try {
        Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();
        while (networkInterfaces.hasMoreElements()) {
          NetworkInterface networkInterface = networkInterfaces.nextElement();
          if (networkInterface == null) continue;

          return NetworkUtils.MAC.formatMACAddress(networkInterface.getHardwareAddress());
        }

        return null; // No MAC address could be retrieved
      } catch (SocketException e) {
        DataLogManager.log(
            String.format(
                "Exception when attempting to retrieve robot MAC address: %s", e.toString()));
        return null;
      }
    }
  }
}
