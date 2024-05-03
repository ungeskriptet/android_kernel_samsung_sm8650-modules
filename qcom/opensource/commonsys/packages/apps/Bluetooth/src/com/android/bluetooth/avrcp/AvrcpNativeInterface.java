/*
 * Copyright 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.android.bluetooth.avrcp;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.util.Log;

import java.util.List;

/**
 * Native Interface to communicate with the JNI layer. This class should never be passed null
 * data.
 */
public class AvrcpNativeInterface {
    private static final String TAG = "AvrcpNativeInterface";
    private static final boolean DEBUG = true;

    private static AvrcpNativeInterface sInstance;
    private AvrcpTargetService mAvrcpService;

    static {
        classInitNative();
    }

    static AvrcpNativeInterface getInterface() {
        if (sInstance == null) {
            sInstance = new AvrcpNativeInterface();
        }

        return sInstance;
    }

    void init(AvrcpTargetService service) {
        d("Init AvrcpNativeInterface");
        mAvrcpService = service;
        initNative();
    }

    void cleanup() {
        d("Cleanup AvrcpNativeInterface");
        mAvrcpService = null;
        cleanupNative();
    }

    Metadata getCurrentSongInfo() {
        d("getCurrentSongInfo");
        if (mAvrcpService == null) {
            Log.w(TAG, "getCurrentSongInfo(): AvrcpTargetService is null");
            return null;
        }

        return mAvrcpService.getCurrentSongInfo();
    }

    PlayStatus getPlayStatus() {
        d("getPlayStatus");
        if (mAvrcpService == null) {
            Log.w(TAG, "getPlayStatus(): AvrcpTargetService is null");
            return null;
        }

        return mAvrcpService.getPlayState();
    }

    void sendMediaKeyEvent(int keyEvent, boolean pushed) {
        d("sendMediaKeyEvent: keyEvent=" + keyEvent + " pushed=" + pushed);
        if (mAvrcpService == null) {
            Log.w(TAG, "sendMediaKeyEvent(): AvrcpTargetService is null");
            return;
        }

        mAvrcpService.sendMediaKeyEvent(keyEvent, pushed);
    }

    String getCurrentMediaId() {
        d("getCurrentMediaId");
        if (mAvrcpService == null) {
            Log.w(TAG, "getMediaPlayerList(): AvrcpTargetService is null");
            return "";
        }

        return mAvrcpService.getCurrentMediaId();
    }

    List<Metadata> getNowPlayingList() {
        d("getNowPlayingList");
        if (mAvrcpService == null) {
            Log.w(TAG, "getMediaPlayerList(): AvrcpTargetService is null");
            return null;
        }

        return mAvrcpService.getNowPlayingList();
    }

    int getCurrentPlayerId() {
        d("getCurrentPlayerId");
        if (mAvrcpService == null) {
            Log.w(TAG, "getMediaPlayerList(): AvrcpTargetService is null");
            return -1;
        }

        return mAvrcpService.getCurrentPlayerId();
    }

    List<PlayerInfo> getMediaPlayerList() {
        d("getMediaPlayerList");
        if (mAvrcpService == null) {
            Log.w(TAG, "getMediaPlayerList(): AvrcpTargetService is null");
            return null;
        }

        return mAvrcpService.getMediaPlayerList();
    }

    // TODO(apanicke): This shouldn't be named setBrowsedPlayer as it doesn't actually connect
    // anything internally. It just returns the number of items in the root folder.
    void setBrowsedPlayer(int playerId) {
        d("setBrowsedPlayer: playerId=" + playerId);
        mAvrcpService.getPlayerRoot(playerId, (a, b, c, d) ->
                setBrowsedPlayerResponse(a, b, c, d));
    }

    void setBrowsedPlayerResponse(int playerId, boolean success, String rootId, int numItems) {
        d("setBrowsedPlayerResponse: playerId=" + playerId
                + " success=" + success
                + " rootId=" + rootId
                + " numItems=" + numItems);
        setBrowsedPlayerResponseNative(playerId, success, rootId, numItems);
    }

    void getFolderItemsRequest(int playerId, String mediaId) {
        d("getFolderItemsRequest: playerId=" + playerId + " mediaId=" + mediaId);
        mAvrcpService.getFolderItems(playerId, mediaId, (a, b) -> getFolderItemsResponse(a, b));
    }

    void getFolderItemsResponse(String parentId, List<ListItem> items) {
        d("getFolderItemsResponse: parentId=" + parentId + " items.size=" + items.size());
        getFolderItemsResponseNative(parentId, items);
    }

    void sendMediaUpdate(boolean metadata, boolean playStatus, boolean queue) {
        d("sendMediaUpdate: metadata=" + metadata
                + " playStatus=" + playStatus
                + " queue=" + queue);
        sendMediaUpdateNative(metadata, playStatus, queue);
    }

    void sendFolderUpdate(boolean availablePlayers, boolean addressedPlayers, boolean uids) {
        d("sendFolderUpdate: availablePlayers=" + availablePlayers
                + " addressedPlayers=" + addressedPlayers
                + " uids=" + uids);
        sendFolderUpdateNative(availablePlayers, addressedPlayers, uids);
    }

    void playItem(int playerId, boolean nowPlaying, String mediaId) {
        d("playItem: playerId=" + playerId + " nowPlaying=" + nowPlaying + " mediaId" + mediaId);
        if (mAvrcpService == null) {
            Log.d(TAG, "playItem: AvrcpTargetService is null");
            return;
        }

        mAvrcpService.playItem(playerId, nowPlaying, mediaId);
    }

    boolean connectDevice(String bdaddr) {
        d("connectDevice: bdaddr=" + bdaddr);
        return connectDeviceNative(bdaddr);
    }

    boolean disconnectDevice(String bdaddr) {
        d("disconnectDevice: bdaddr=" + bdaddr);
        return disconnectDeviceNative(bdaddr);
    }

    void setActiveDevice(String bdaddr) {
        BluetoothDevice device =
                BluetoothAdapter.getDefaultAdapter().getRemoteDevice(bdaddr.toUpperCase());
        d("setActiveDevice: device=" + device);
        mAvrcpService.setActiveDevice(device);
    }

    void deviceConnected(String bdaddr, boolean absoluteVolume) {
        BluetoothDevice device =
                BluetoothAdapter.getDefaultAdapter().getRemoteDevice(bdaddr.toUpperCase());
        d("deviceConnected: device=" + device + " absoluteVolume=" + absoluteVolume);
        if (mAvrcpService == null) {
            Log.w(TAG, "deviceConnected: AvrcpTargetService is null");
            return;
        }

        mAvrcpService.deviceConnected(device, absoluteVolume);
    }

    void deviceDisconnected(String bdaddr) {
        BluetoothDevice device =
                BluetoothAdapter.getDefaultAdapter().getRemoteDevice(bdaddr.toUpperCase());
        d("deviceDisconnected: device=" + device);
        if (mAvrcpService == null) {
            Log.w(TAG, "deviceDisconnected: AvrcpTargetService is null");
            return;
        }

        mAvrcpService.deviceDisconnected(device);
    }

    void sendVolumeChanged(int volume) {
        d("sendVolumeChanged: volume=" + volume);
        sendVolumeChangedNative(volume);
    }

    void setVolume(int volume) {
        d("setVolume: volume=" + volume);
        if (mAvrcpService == null) {
            Log.w(TAG, "setVolume: AvrcpTargetService is null");
            return;
        }

        mAvrcpService.setVolume(volume);
    }

    /** Request from remote to list supported values for player setting. */
    void listPlayerSettingValuesRequest(byte settingRequest) {
        byte[] valuesArray;
        // TODO remove stub changes with implementation
        final int STATE_DEFAULT_OFF = 1;
        valuesArray = new byte[1];
        valuesArray[0] = STATE_DEFAULT_OFF;
        listPlayerSettingValuesResponseNative(
                settingRequest, valuesArray);
    }

    /** Request from remote current values for player settings. */
    void getCurrentPlayerSettingValuesRequest(byte[] settingsRequest) {
        // TODO remove stub changes with implementation
        final int STATE_DEFAULT_OFF = 1;
        byte[] valuesArray = new byte[settingsRequest.length];
        for (int i = 0; i < settingsRequest.length; i++) {
            valuesArray[i] = (byte) STATE_DEFAULT_OFF;
        }
        getPlayerSettingsResponseNative(settingsRequest, valuesArray);
    }

    /** Request from remote to set current values for player settings. */
    void setPlayerSettingsRequest(byte[] settingsRequest, byte[] valuesRequest) {
        // TODO remove stub changes with implementation
        boolean success = false;
        setPlayerSettingsResponseNative(success);
    }

    void sendPlayerSettings(int repeatMode, int shuffleMode) {
        final int SETTING_REPEAT = 2;
        final int SETTING_SHUFFLE = 3;
        byte[] settingsArray = new byte[2];
        byte[] valuesArray = new byte[2];
        settingsArray[0] = (byte) SETTING_REPEAT;
        settingsArray[1] = (byte) SETTING_SHUFFLE;
        valuesArray[0] = (byte) repeatMode;
        valuesArray[1] = (byte) shuffleMode;
        sendPlayerSettingsNative(settingsArray, valuesArray);
    }

    private static native void classInitNative();
    private native void initNative();
    private native void sendMediaUpdateNative(
            boolean trackChanged, boolean playState, boolean playPos);
    private native void sendFolderUpdateNative(
            boolean availablePlayers, boolean addressedPlayers, boolean uids);
    private native void setBrowsedPlayerResponseNative(
            int playerId, boolean success, String rootId, int numItems);
    private native void getFolderItemsResponseNative(String parentId, List<ListItem> list);
    private native void cleanupNative();
    private native boolean connectDeviceNative(String bdaddr);
    private native boolean disconnectDeviceNative(String bdaddr);
    private native void sendVolumeChangedNative(int volume);
    private native void setBipClientStatusNative(String bdaddr, boolean connected);
    private native void listPlayerSettingsResponseNative(byte[] attributes);
    private native void listPlayerSettingValuesResponseNative(byte attribute, byte[] values);
    private native void getPlayerSettingsResponseNative(byte[] attributes, byte[] values);
    private native void setPlayerSettingsResponseNative(boolean success);
    private native void sendPlayerSettingsNative(byte[] attributes, byte[] values);

    private static void d(String msg) {
        if (DEBUG) {
            Log.d(TAG, msg);
        }
    }
}
