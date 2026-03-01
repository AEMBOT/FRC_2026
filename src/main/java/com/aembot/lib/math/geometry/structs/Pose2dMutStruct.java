package com.aembot.lib.math.geometry.structs;

import com.aembot.lib.math.geometry.Pose2dMut;
import com.aembot.lib.math.geometry.Rotation2dMut;
import com.aembot.lib.math.geometry.Translation2dMut;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class Pose2dMutStruct implements Struct<Pose2dMut> {
  @Override
  public Class<Pose2dMut> getTypeClass() {
    return Pose2dMut.class;
  }

  @Override
  public String getTypeName() {
    return "Pose2dMut";
  }

  @Override
  public int getSize() {
    return Translation2dMut.struct.getSize() + Rotation2dMut.struct.getSize();
  }

  @Override
  public String getSchema() {
    return "Translation2dMut translation;Rotation2dMut rotation";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Translation2dMut.struct, Rotation2dMut.struct};
  }

  @Override
  public Pose2dMut unpack(ByteBuffer bb) {
    Translation2dMut translation = Translation2dMut.struct.unpack(bb);
    Rotation2dMut rotation = Rotation2dMut.struct.unpack(bb);
    return new Pose2dMut(translation, rotation);
  }

  @Override
  public void pack(ByteBuffer bb, Pose2dMut value) {
    Translation2dMut.struct.pack(bb, value.getTranslation());
    Rotation2dMut.struct.pack(bb, value.getRotation());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
